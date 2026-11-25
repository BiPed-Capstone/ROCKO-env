#ifndef ROCKO_ENV_ICM20948
#define ROCKO_ENV_ICM20948

// The IMU hardware plugin. The part fitted is a BNO055 (the ICM20948 name is historical, see
// CLAUDE.md section 10). Since 2026-11-24 the plugin reads the sensor itself, over I2C, with one
// bounded 12-byte burst per control cycle (finding C-1; it used to wait with no time limit on a
// Python service, ICM20948.py, now removed). How it works: docs/architecture/imu-i2c-read.md.

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rocko_env/Bno055UartProtocol.hpp"
#include "rocko_env/ImuFaultTracker.hpp"
#include "rocko_env/ImuSnapshotCheck.hpp"

namespace rocko_env
{
class ICM20948 : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948);

  ~ICM20948() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Opens the I2C bus and starts the sensor (reset, NDOF fusion mode), then waits for its first
  // believable sample. Takes about a second. Never called from the control loop.
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // One I2C read of `length` registers starting at `first_register`, as one combined transaction
  // (register address written, then a repeated start and the read). Returns false on any bus error,
  // including the kernel's timeout. Bounded by kI2cTimeoutMs.
  bool read_registers(std::uint8_t first_register, std::uint8_t * out, std::uint16_t length);
  bool write_register(std::uint8_t reg, std::uint8_t value);
  // Reads the burst and judges it (range limits, then the pitch-jump check against the last good
  // sample, `elapsed_s` seconds old). On success updates the four exported values and returns kOk.
  SnapshotVerdict read_snapshot(double elapsed_s);
  void close_bus();

  std::string YAW_KEY = "yaw";
  std::string PITCH_KEY = "pitch";
  std::string ROLL_KEY = "roll";
  std::string PITCH_RATE_KEY = "pitch_rate";

  // ---- Hardware parameters (the <param> tags of the Gyro block in rocko.ros2_control.xacro) ----
  std::string _i2c_device = "/dev/i2c-1";  // the Pi's I2C bus on the 40-pin header (pins 3 and 5)
  int _i2c_address = 0x28;                 // the BNO055's default address (Adafruit's default too)
  // Overwritten from the xacro in on_init. The pitch offset has NO default (NaN): it is a measurement of this robot
  // (pitch_offset_deg in the xacro), and on_init refuses to start without a finite one rather than guess.
  ImuMounting _mounting{std::numeric_limits<double>::quiet_NaN(), 0, -1};

  int _i2c_fd = -1;  // the open bus, or -1

  // Kernel timeout for one I2C transaction [ms]. PROPOSED: 10 ms, one control cycle. The I2C_TIMEOUT
  // ioctl counts in units of 10 ms, so on_configure passes kI2cTimeoutMs / 10. A normal 12-byte read takes about 1.5 ms at the Pi's default
  // 100 kHz; a read that times out costs one overrun cycle and counts as a failed read. Note that
  // the setting is for the whole bus (/dev/i2c-1), not only this device. UNVERIFIED on the Pi, and
  // possibly NOT a hard bound (2026-11-24 realtime audit, from memory of the kernel source):
  //  - Pi 4 and earlier (i2c-bcm2835): waits on this timeout, but with a 100 Hz kernel tick 10 ms is
  //    one jiffy, which can fire anywhere in 0..10 ms and fail a healthy read at random;
  //  - Pi 5 (designware on RP1): may poll up to a fixed 20 ms for a busy bus before the transfer, so
  //    the worst case can exceed 30 ms.
  // read() measures every transaction and logs the maximum (see kTimingReportReads): check it on the
  // robot, with `grep CONFIG_HZ= /boot/config-$(uname -r)` and the Pi model.
  static constexpr int kI2cTimeoutMs = 10;  // [ms]; a multiple of 10
  static_assert(kI2cTimeoutMs >= 10 && kI2cTimeoutMs % 10 == 0, "I2C_TIMEOUT counts in 10 ms units");

  // SAFETY PARAMETERS. Proposed values; a human confirms them on the robot. Edit-and-rebuild
  // constants, not ROS parameters.
  //
  // Finding C-2 (docs/claude-analysis/balance-loop-findings.md): read() returns ERROR once this many
  // reads in a row have failed: 5 cycles x 10 ms = 50 ms at update_rate 100. A failed read now
  // returns within kI2cTimeoutMs (finding C-1), so this is also close to the wall-clock time.
  // What counts as a failed read for the fault tracker: a bus error or timeout, OR a sample that
  // check_imu_snapshot() rejects. That includes a bad pitch RATE: the angles and the rate come from
  // one transaction, so a garbage rate makes the whole sample suspect. (Until 2026-11-24 a bad rate
  // was only held and logged and never reached the fault tracker; that gap had to be closed before
  // a controller uses pitch_rate, finding H-6.)
  static constexpr int kMaxConsecutiveImuFailures = 5;  // [reads] = [control cycles]
  // read() also returns ERROR when this many of the last ImuFaultTracker::kWindowReads reads
  // failed, which catches a sensor that alternates good and bad reads. Tracker default.
  static constexpr int kMaxWindowImuFailures = ImuFaultTracker::kMaxWindowFailures;  // [reads]
  ImuFaultTracker _imu_fault_tracker{kMaxConsecutiveImuFailures, kMaxWindowImuFailures};
  static_assert(
    kMaxConsecutiveImuFailures > 0 && kMaxWindowImuFailures > 0 &&
      kMaxWindowImuFailures <= ImuFaultTracker::kWindowReads,
    "fault limits must be positive, and the windowed limit must fit in the window");

  // Exported values, in the units of ImuReading (degrees and deg/s; pitch includes the offset).
  // NaN until the first good read, so a failure before then can never look like 0.0 (upright).
  // Held, never zeroed, on a failed read (finding C-2); the fault tracker decides when holding has
  // gone on too long and read() must return ERROR.
  double _yaw = std::numeric_limits<double>::quiet_NaN();
  double _pitch = std::numeric_limits<double>::quiet_NaN();
  double _roll = std::numeric_limits<double>::quiet_NaN();
  double _pitch_rate = std::numeric_limits<double>::quiet_NaN();

  // [s] time since the last good sample, summed from read()'s period; the pitch-jump check uses it.
  double _seconds_since_good = 1e9;  // "very long": no good sample yet, nothing to compare with

  // Timing of the I2C transaction in read(), reported (INFO) once every kTimingReportReads reads:
  // 3000 = 30 s at 100 Hz. Not a throttle macro, so each report covers exactly that many reads.
  static constexpr int kTimingReportReads = 3000;
  int _timing_reads = 0;
  int _timing_failures = 0;
  double _timing_max_us = 0.0;  // [us] slowest transaction in the current report window

  // The 12 burst bytes (registers 0x14..0x1F). A member, so read() allocates nothing.
  std::array<std::uint8_t, bno055_uart::kBurstLength> _burst{};

  rclcpp::Logger _logger = rclcpp::get_logger("ICM20948");
  // Steady clock for the throttled logs: no ROS time, no node needed.
  rclcpp::Clock _log_clock{RCL_STEADY_TIME};
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_ICM20948
