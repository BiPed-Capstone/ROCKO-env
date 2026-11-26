#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rocko_env/BNO055.hpp"

namespace rocko_env
{

namespace
{
// Minimum time between repeats of the same throttled log line in this file.
constexpr int kLogThrottleMs = 1000;  // [ms]

// BNO055 registers and values used to start the sensor. They are the ones Adafruit's
// adafruit_bno055 driver uses (the driver the removed Python IMU node ran, and the one the 2026-11-23/24 axis and sign
// measurements were taken with), written from memory of that driver and the datasheet: UNVERIFIED
// against the datasheet in this change. The first on-robot run checks them (the chip id check below
// fails loudly if the address or bus is wrong).
constexpr std::uint8_t kRegChipId = 0x00;      // CHIP_ID, reads 0xA0
constexpr std::uint8_t kChipId = 0xA0;
constexpr std::uint8_t kRegPageId = 0x07;      // PAGE_ID: register map page 0 or 1
constexpr std::uint8_t kRegOprMode = 0x3D;     // OPR_MODE
constexpr std::uint8_t kRegPwrMode = 0x3E;     // PWR_MODE
constexpr std::uint8_t kRegSysTrigger = 0x3F;  // SYS_TRIGGER; bit 5 = reset
constexpr std::uint8_t kModeConfig = 0x00;     // OPR_MODE CONFIGMODE
constexpr std::uint8_t kModeNdof = 0x0C;       // OPR_MODE NDOF: 9-axis fusion, what Adafruit's driver selects
constexpr std::uint8_t kPowerNormal = 0x00;
constexpr std::uint8_t kTriggerReset = 0x20;

// Start-up waits (configure only, never in read()). The datasheet's mode-switch times are about
// 7 ms into a fusion mode and 19 ms into CONFIG; the reset takes about 650 ms. The waits below are
// the driver's, with margin.
constexpr auto kModeSwitchWait = std::chrono::milliseconds(30);
constexpr auto kResetWait = std::chrono::milliseconds(700);
constexpr auto kBootRetryWait = std::chrono::milliseconds(1000);  // chip id wrong at power-up: wait once
// After NDOF is selected the sensor returns all zeros for a while (tools/imu_tilt_log.py saw it).
// Configure waits at most this long for a first believable sample, polling every cycle.
constexpr auto kFirstSampleTimeout = std::chrono::milliseconds(3000);
constexpr auto kFirstSamplePoll = std::chrono::milliseconds(10);

double parameter_or(
  const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : std::stod(it->second);
}

// A whole number, or an exception: "0.9" must not quietly become axis 0, nor "-1.5" sign -1.
int whole_parameter_or(
  const hardware_interface::HardwareInfo & info, const std::string & name, int fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  std::size_t used = 0;
  const int value = std::stoi(it->second, &used);
  if (used != it->second.size()) {
    throw std::invalid_argument(name + " must be a whole number, got '" + it->second + "'");
  }
  return value;
}
}  // namespace

BNO055::~BNO055()
{
  close_bus();
}

hardware_interface::CallbackReturn BNO055::on_init(
    const hardware_interface::HardwareInfo & info)
{
  // Parameters are read from `info` directly, as SimPlant does. (Calling the base on_init(info) is
  // deprecated in this Jazzy ros2_control, and the framework stores the info itself.)
  // Hardware parameters from the xacro. Missing ones keep the defaults in BNO055.hpp, except
  // pitch_offset_deg, which has none (NaN) and so must be given; a value that does not parse, or a mounting
  // that check_imu_snapshot() would reject, refuses to start.
  try {
    const auto device = info.hardware_parameters.find("i2c_device");
    if (device != info.hardware_parameters.end()) {
      _i2c_device = device->second;
    }
    _i2c_address = static_cast<int>(parameter_or(info, "i2c_address", _i2c_address));
    _mounting.pitch_offset_deg = parameter_or(info, "pitch_offset_deg", _mounting.pitch_offset_deg);
    _mounting.pitch_rate_axis = whole_parameter_or(info, "pitch_rate_axis", _mounting.pitch_rate_axis);
    _mounting.pitch_rate_sign = whole_parameter_or(info, "pitch_rate_sign", _mounting.pitch_rate_sign);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(_logger, "A Gyro hardware parameter is not a valid number: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (_mounting.pitch_rate_axis < 0 || _mounting.pitch_rate_axis > 2 ||
    (_mounting.pitch_rate_sign != 1 && _mounting.pitch_rate_sign != -1) ||
    !std::isfinite(_mounting.pitch_offset_deg))
  {
    RCLCPP_ERROR(
      _logger, "pitch_rate_axis must be 0, 1 or 2 and pitch_rate_sign 1 or -1 (got %d, %d), "
      "and pitch_offset_deg must be given in the xacro, finite (got %.2f)", _mounting.pitch_rate_axis,
      _mounting.pitch_rate_sign, _mounting.pitch_offset_deg);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    _logger, "BNO055 on %s address 0x%02x; pitch offset %+.2f deg; pitch rate = %+d * gyro[%d]",
    _i2c_device.c_str(), _i2c_address, _mounting.pitch_offset_deg, _mounting.pitch_rate_sign,
    _mounting.pitch_rate_axis);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055::on_configure(const rclcpp_lifecycle::State &)
{
  close_bus();
  // A fresh start forgets the failures of the last run (an IMU fault, then cleanup and configure).
  _imu_fault_tracker = ImuFaultTracker{kMaxConsecutiveImuFailures, kMaxWindowImuFailures};
  _seconds_since_good = 1e9;
  _i2c_fd = ::open(_i2c_device.c_str(), O_RDWR | O_CLOEXEC);
  if (_i2c_fd < 0) {
    RCLCPP_ERROR(
      _logger, "Cannot open %s: %s. Is I2C enabled, and is this running as root?",
      _i2c_device.c_str(), std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Bound every transaction (finding C-1), and no automatic retries that would multiply the bound.
  if (::ioctl(_i2c_fd, I2C_TIMEOUT, kI2cTimeoutMs / 10) < 0 || ::ioctl(_i2c_fd, I2C_RETRIES, 0) < 0) {
    RCLCPP_ERROR(_logger, "Cannot set the I2C timeout on %s: %s", _i2c_device.c_str(),
      std::strerror(errno));
    close_bus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Is a BNO055 there? It can miss the first read just after power-up, so try twice.
  std::uint8_t chip_id = 0;
  if (!read_registers(kRegChipId, &chip_id, 1) || chip_id != kChipId) {
    std::this_thread::sleep_for(kBootRetryWait);
    if (!read_registers(kRegChipId, &chip_id, 1) || chip_id != kChipId) {
      RCLCPP_ERROR(
        _logger, "No BNO055 at address 0x%02x on %s (chip id 0x%02x, expected 0x%02x)",
        _i2c_address, _i2c_device.c_str(), chip_id, kChipId);
      close_bus();
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // The same start as Adafruit's driver, so the sensor is in the state the axis and sign
  // measurements were taken in: CONFIG mode, reset (which also restores the default units and
  // orientation convention), normal power, page 0, then NDOF fusion.
  write_register(kRegOprMode, kModeConfig);
  std::this_thread::sleep_for(kModeSwitchWait);
  write_register(kRegSysTrigger, kTriggerReset);  // the chip resets mid-write and may not ack it
  std::this_thread::sleep_for(kResetWait);
  if (!read_registers(kRegChipId, &chip_id, 1) || chip_id != kChipId) {
    std::this_thread::sleep_for(kBootRetryWait);  // a slow reset: one more chance
  }
  bool ok = write_register(kRegPwrMode, kPowerNormal);
  ok = ok && write_register(kRegPageId, 0x00);
  ok = ok && write_register(kRegSysTrigger, 0x00);
  std::this_thread::sleep_for(kModeSwitchWait);
  ok = ok && write_register(kRegOprMode, kModeNdof);
  std::this_thread::sleep_for(kModeSwitchWait);
  if (!ok) {
    RCLCPP_ERROR(_logger, "BNO055 start-up writes failed on %s: %s", _i2c_device.c_str(),
      std::strerror(errno));
    close_bus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Wait for the fusion to produce a believable sample, so the controllers never start on NaN
  // because the sensor was still warming up. TWO good samples in a row, the second checked against
  // the first (pitch_step_plausible): a single corrupted first sample would otherwise become the
  // reference every later sample is compared with.
  const auto deadline = std::chrono::steady_clock::now() + kFirstSampleTimeout;
  const double poll_s = std::chrono::duration<double>(kFirstSamplePoll).count();
  SnapshotVerdict verdict = SnapshotVerdict::kNotDecoded;
  int good_in_a_row = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    // The first of the pair has nothing to be compared with (a huge gap skips the check).
    verdict = read_snapshot(good_in_a_row == 0 ? 1e9 : poll_s);
    good_in_a_row = (verdict == SnapshotVerdict::kOk) ? good_in_a_row + 1 : 0;
    if (verdict != SnapshotVerdict::kOk) {
      // A failed pair: forget the first one, so the next attempt starts a fresh pair.
      _pitch = _pitch_rate = std::numeric_limits<double>::quiet_NaN();
    }
    if (good_in_a_row >= 2) {
      _seconds_since_good = 0.0;
      RCLCPP_INFO(
        _logger, "BNO055 ready: pitch %+.2f deg, roll %+.2f deg, pitch rate %+.2f deg/s",
        _pitch, _roll, _pitch_rate);
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    std::this_thread::sleep_for(kFirstSamplePoll);
  }
  // No believable sample at all: a sensor fault found before any controller starts. Refuse to
  // configure, rather than start the balance loop on NaN.
  RCLCPP_ERROR(
    _logger, "BNO055 gave no believable sample within %lld ms of start-up (last: %s)",
    static_cast<long long>(kFirstSampleTimeout.count()), to_string(verdict));
  close_bus();
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn BNO055::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_bus();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO055::on_shutdown(const rclcpp_lifecycle::State &)
{
  close_bus();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void BNO055::close_bus()
{
  if (_i2c_fd >= 0) {
    ::close(_i2c_fd);
    _i2c_fd = -1;
  }
}

bool BNO055::read_registers(std::uint8_t first_register, std::uint8_t * out, std::uint16_t length)
{
  if (_i2c_fd < 0) {
    return false;
  }
  std::uint8_t reg = first_register;
  i2c_msg messages[2];
  messages[0].addr = static_cast<__u16>(_i2c_address);
  messages[0].flags = 0;  // write the register address...
  messages[0].len = 1;
  messages[0].buf = &reg;
  messages[1].addr = static_cast<__u16>(_i2c_address);
  messages[1].flags = I2C_M_RD;  // ...then, after a repeated start, read `length` bytes from it
  messages[1].len = length;
  messages[1].buf = out;
  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = messages;
  transfer.nmsgs = 2;
  // Returns the number of messages done, 2, or -1 on a NACK, a bus error or the timeout.
  return ::ioctl(_i2c_fd, I2C_RDWR, &transfer) == 2;
}

bool BNO055::write_register(std::uint8_t reg, std::uint8_t value)
{
  if (_i2c_fd < 0) {
    return false;
  }
  std::uint8_t bytes[2] = {reg, value};
  i2c_msg message;
  message.addr = static_cast<__u16>(_i2c_address);
  message.flags = 0;
  message.len = 2;
  message.buf = bytes;
  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = &message;
  transfer.nmsgs = 1;
  return ::ioctl(_i2c_fd, I2C_RDWR, &transfer) == 1;
}

SnapshotVerdict BNO055::read_snapshot(double elapsed_s)
{
  // ONE transaction for gyro and Euler angles (registers 0x14..0x1F): the rate and the angle are
  // (The decoder lives in Bno055UartProtocol.hpp; the register bytes are the same on I2C, see its STATUS note.)
  // the same instant (finding H-3), and there is one bounded wait per cycle, not three.
  if (!read_registers(bno055_uart::kBurstFirstRegister, _burst.data(), bno055_uart::kBurstLength)) {
    return SnapshotVerdict::kNotDecoded;
  }
  const auto snapshot = bno055_uart::decode_gyro_euler(_burst.data(), _burst.size());
  ImuReading reading;
  SnapshotVerdict verdict = check_imu_snapshot(snapshot, _mounting, reading);
  // I2C has no checksum: a flipped high byte gives an in-range but false pitch. The gyro bounds how
  // far pitch can really have moved since the last good sample (ImuSnapshotCheck.hpp).
  if (verdict == SnapshotVerdict::kOk &&
    !pitch_step_plausible(_pitch, _pitch_rate, reading.pitch_deg, reading.pitch_rate_deg_s, elapsed_s))
  {
    verdict = SnapshotVerdict::kPitchJump;
  }
  if (verdict == SnapshotVerdict::kOk) {
    _yaw = reading.yaw_deg;
    _pitch = reading.pitch_deg;
    _roll = reading.roll_deg;
    _pitch_rate = reading.pitch_rate_deg_s;
  }
  return verdict;
}

std::vector<hardware_interface::StateInterface> BNO055::export_state_interfaces()
{
  // Export the member level vars we saved data to in read()
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", YAW_KEY, &_yaw));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", PITCH_KEY, &_pitch));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", ROLL_KEY, &_roll));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "left_wheel_joint", PITCH_RATE_KEY, &_pitch_rate));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", YAW_KEY, &_yaw));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", PITCH_KEY, &_pitch));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", ROLL_KEY, &_roll));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "right_wheel_joint", PITCH_RATE_KEY, &_pitch_rate));

  return state_interfaces;
}

// Reads yaw, pitch and roll [deg] and pitch rate [deg/s] from the BNO055, once per control cycle.
//
// Real-time notes (CLAUDE.md section 6): one ioctl, bounded by kI2cTimeoutMs (finding C-1); no
// allocation (the buffer is a member); logs only throttled.
//
// On a failed read the last good values are kept: never 0.0, because pitch 0.0 is the balanced
// setpoint and a failure that looked like 0.0 would tell the pitch controller "level" while the
// robot is blind (finding C-2). Failures are counted; read() returns ERROR once _imu_fault_tracker
// trips. A single dropped sample returns OK on purpose.
//
// The angles and the pitch rate are judged TOGETHER (check_imu_snapshot): a bad rate is a failed
// read and counts in the fault tracker exactly like a bad angle, because both come from the one
// transaction.
hardware_interface::return_type BNO055::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  _seconds_since_good += period.seconds();
  const auto start = std::chrono::steady_clock::now();
  const SnapshotVerdict verdict = read_snapshot(_seconds_since_good);
  const double took_us =
    std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
  const bool good_read = (verdict == SnapshotVerdict::kOk);
  if (good_read) {
    _seconds_since_good = 0.0;
  }

  // Timing report: how long the bounded I2C read really takes on this Pi (the 10 ms bound is
  // unverified, see kI2cTimeoutMs). One INFO line per kTimingReportReads reads.
  _timing_max_us = std::max(_timing_max_us, took_us);
  _timing_failures += good_read ? 0 : 1;
  if (++_timing_reads >= kTimingReportReads) {
    RCLCPP_INFO(
      _logger, "IMU read timing: slowest %.0f us of the last %d reads, %d failed",
      _timing_max_us, _timing_reads, _timing_failures);
    _timing_reads = 0;
    _timing_failures = 0;
    _timing_max_us = 0.0;
  }

  _imu_fault_tracker.record(good_read);
  if (good_read) {
    return hardware_interface::return_type::OK;
  }

  RCLCPP_ERROR_THROTTLE(
    _logger, _log_clock, kLogThrottleMs,
    "IMU read failed (%s): %d failed in a row, %d of the last %d reads, %lu in total. "
    "Holding the last good values.",
    to_string(verdict), _imu_fault_tracker.consecutive_failures(),
    _imu_fault_tracker.window_failures(), ImuFaultTracker::kWindowReads,
    _imu_fault_tracker.total_failures());

  if (_imu_fault_tracker.tripped()) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rocko_env

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rocko_env::BNO055, hardware_interface::SensorInterface)
