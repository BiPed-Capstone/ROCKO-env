// Host-side self-test for rocko_env::check_imu_snapshot (ImuSnapshotCheck.hpp): the rule that turns
// one 12-byte BNO055 burst into the values the IMU plugin exports, or rejects it (findings C-1, C-2,
// H-3, H-6). No ROS, no I2C; it runs on a laptop.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/bno055/include \
//       tools/imu_snapshot_check_test.cpp -o /tmp/imu_snapshot_check_test && /tmp/imu_snapshot_check_test
// Exit status 0 and "all ImuSnapshotCheck tests passed" mean success. This proves the decision on
// made-up bytes only: not the I2C transaction, the BNO055's register map, or anything on the robot.

#undef NDEBUG  // asserts must stay on even if someone adds -DNDEBUG
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "rocko_env/ImuSnapshotCheck.hpp"

using rocko_env::ImuMounting;
using rocko_env::ImuReading;
using rocko_env::SnapshotVerdict;
using rocko_env::check_imu_snapshot;
namespace uart = rocko_env::bno055_uart;

namespace
{
// A mounting like the robot's (gyro X, sign -1); the offset 7.6 deg is the old xacro value (now -2.5), kept here
// because the range cases below are written around it.
const ImuMounting kRobot{7.6, 0, -1};

// Put a value [units] into the 12 burst bytes at `register_lsb`, 16 LSB per unit, little-endian.
void put(std::uint8_t * burst, std::uint8_t register_lsb, double value)
{
  const int raw = static_cast<int>(std::lround(value * 16.0));
  const int u = raw < 0 ? raw + 65536 : raw;
  burst[register_lsb - uart::kBurstFirstRegister] = static_cast<std::uint8_t>(u & 0xFF);
  burst[register_lsb - uart::kBurstFirstRegister + 1] = static_cast<std::uint8_t>((u >> 8) & 0xFF);
}

uart::GyroEulerSnapshot burst(double gx, double gy, double gz, double heading, double roll, double pitch)
{
  std::uint8_t b[uart::kBurstLength] = {};
  put(b, uart::kRegGyroXLsb, gx);
  put(b, uart::kRegGyroYLsb, gy);
  put(b, uart::kRegGyroZLsb, gz);
  put(b, uart::kRegEulerHeadingLsb, heading);
  put(b, uart::kRegEulerRollLsb, roll);
  put(b, uart::kRegEulerPitchLsb, pitch);
  return uart::decode_gyro_euler(b, sizeof(b));
}

bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

void test_good_sample_applies_offset_axis_and_sign()
{
  ImuReading r;
  assert(check_imu_snapshot(burst(12.5, -3.0, 1.0, 90.0, 2.0, -4.0), kRobot, r) == SnapshotVerdict::kOk);
  assert(near(r.yaw_deg, 90.0));
  assert(near(r.roll_deg, 2.0));
  assert(near(r.pitch_deg, -4.0 + 7.6));       // offset ADDED
  assert(near(r.pitch_rate_deg_s, -12.5));     // sign -1 times gyro X
  // another axis and sign
  assert(check_imu_snapshot(burst(12.5, -3.0, 1.0, 90.0, 2.0, -4.0), ImuMounting{0.0, 1, 1}, r) ==
    SnapshotVerdict::kOk);
  assert(near(r.pitch_rate_deg_s, -3.0) && near(r.pitch_deg, -4.0));
}

void test_still_gyro_is_a_real_measurement()
{
  ImuReading r;
  assert(check_imu_snapshot(burst(0.0, 0.0, 0.0, 10.0, 1.0, 3.0), kRobot, r) == SnapshotVerdict::kOk);
  assert(near(r.pitch_rate_deg_s, 0.0));
}

void test_all_zero_start_up_block_is_rejected_and_output_untouched()
{
  ImuReading r{1.0, 2.0, 3.0, 4.0};
  assert(check_imu_snapshot(burst(0, 0, 0, 0, 0, 0), kRobot, r) == SnapshotVerdict::kAllAnglesEqual);
  assert(r.yaw_deg == 1.0 && r.roll_deg == 2.0 && r.pitch_deg == 3.0 && r.pitch_rate_deg_s == 4.0);
  // equal but non-zero is rejected too
  assert(check_imu_snapshot(burst(1, 2, 3, 5.0, 5.0, 5.0), kRobot, r) == SnapshotVerdict::kAllAnglesEqual);
}

void test_not_decoded_is_rejected()
{
  ImuReading r;
  const auto s = uart::decode_gyro_euler(nullptr, 0);  // all NaN, invalid
  assert(check_imu_snapshot(s, kRobot, r) == SnapshotVerdict::kNotDecoded);
}

void test_angle_limits()
{
  ImuReading r;
  // pitch 175 + offset 7.6 = 182.6 > 180: rejected
  assert(check_imu_snapshot(burst(0, 0, 0, 10, 1, 175.0), kRobot, r) == SnapshotVerdict::kAngleOutOfRange);
  // pitch 172 + 7.6 = 179.6: accepted
  assert(check_imu_snapshot(burst(0, 0, 0, 10, 1, 172.0), kRobot, r) == SnapshotVerdict::kOk);
  assert(check_imu_snapshot(burst(0, 0, 0, 10, 181.0, 1), kRobot, r) == SnapshotVerdict::kAngleOutOfRange);
  assert(check_imu_snapshot(burst(0, 0, 0, 361.0, 1, 2), kRobot, r) == SnapshotVerdict::kAngleOutOfRange);
}

void test_rate_beyond_full_scale_rejects_whole_sample()
{
  ImuReading r{1.0, 2.0, 3.0, 4.0};
  // 2047.9 deg/s fits in int16 at 16 LSB per deg/s and is beyond the 2000 deg/s full scale
  assert(check_imu_snapshot(burst(2047.0, 0, 0, 10, 1, 2), kRobot, r) == SnapshotVerdict::kRateOutOfRange);
  assert(r.pitch_deg == 3.0);  // the good-looking angles are NOT taken from a suspect transaction
  // a wild rate on an axis that is NOT the pitch axis does not matter
  assert(check_imu_snapshot(burst(1.0, 2047.0, 0, 10, 1, 2), kRobot, r) == SnapshotVerdict::kOk);
  // 2000 exactly is accepted
  assert(check_imu_snapshot(burst(-2000.0, 0, 0, 10, 1, 2), kRobot, r) == SnapshotVerdict::kOk);
  assert(near(r.pitch_rate_deg_s, 2000.0));
}

void test_bad_mounting_is_refused()
{
  ImuReading r;
  const auto good = burst(1, 2, 3, 10, 1, 2);
  assert(check_imu_snapshot(good, ImuMounting{0.0, 3, 1}, r) == SnapshotVerdict::kBadMounting);
  assert(check_imu_snapshot(good, ImuMounting{0.0, -1, 1}, r) == SnapshotVerdict::kBadMounting);
  assert(check_imu_snapshot(good, ImuMounting{0.0, 0, 0}, r) == SnapshotVerdict::kBadMounting);
  assert(check_imu_snapshot(good, ImuMounting{0.0, 0, 2}, r) == SnapshotVerdict::kBadMounting);
}

void test_pitch_jump()
{
  using rocko_env::pitch_step_plausible;
  // still robot, one cycle: a 32 deg step (a flipped high byte) is rejected, 1 deg is fine
  assert(!pitch_step_plausible(3.0, 0.0, 35.0, 0.0, 0.01));
  assert(pitch_step_plausible(3.0, 0.0, 4.0, 0.0, 0.01));
  // the allowance at 10 ms from rest: 5 deg slack + 0.5 x 20000 x 0.01^2 = 1 deg -> 6 deg passes, 6.1 does not
  assert(pitch_step_plausible(0.0, 0.0, 6.0, 0.0, 0.01));
  assert(!pitch_step_plausible(0.0, 0.0, 6.1, 0.0, 0.01));
  // review round 3: a real 300 deg/s step seen after one rejected cycle (20 ms, stale rate 0) passes
  assert(pitch_step_plausible(0.0, 0.0, 6.0, 300.0, 0.02));
  // a real fast fall: 300 deg/s for 20 ms = 6 deg + slack, rate steady
  assert(pitch_step_plausible(10.0, 300.0, 18.0, 300.0, 0.02));
  // the allowance comes from the LAST rate only: a wild new rate cannot widen it
  // (240 deg/s is a legal rate step, so only the pitch allowance decides: 7 deg > 6 deg allowed)
  assert(!pitch_step_plausible(10.0, 0.0, 17.0, 240.0, 0.01));
  // rate steps: 200 deg/s in 10 ms passes (allowance 250), a flipped bit 8 (+256) does not
  assert(pitch_step_plausible(0.0, 10.0, 0.0, 210.0, 0.01));
  assert(!pitch_step_plausible(0.0, 10.0, 0.0, 266.0, 0.01));
  // after a long gap, or with no earlier sample, nothing to compare: accepted
  assert(pitch_step_plausible(3.0, 0.0, 60.0, 0.0, 0.2));
  assert(pitch_step_plausible(std::nan(""), std::nan(""), 60.0, 0.0, 0.01));
}

void test_every_verdict_has_a_name()
{
  for (auto v : {SnapshotVerdict::kOk, SnapshotVerdict::kNotDecoded, SnapshotVerdict::kAllAnglesEqual,
      SnapshotVerdict::kAngleOutOfRange, SnapshotVerdict::kRateOutOfRange, SnapshotVerdict::kBadMounting,
      SnapshotVerdict::kPitchJump})
  {
    assert(std::string_view(rocko_env::to_string(v)) != "unknown");
  }
}
}  // namespace

int main()
{
  test_good_sample_applies_offset_axis_and_sign();
  test_still_gyro_is_a_real_measurement();
  test_all_zero_start_up_block_is_rejected_and_output_untouched();
  test_not_decoded_is_rejected();
  test_angle_limits();
  test_rate_beyond_full_scale_rejects_whole_sample();
  test_bad_mounting_is_refused();
  test_pitch_jump();
  test_every_verdict_has_a_name();
  std::puts("all ImuSnapshotCheck tests passed");
  return 0;
}
