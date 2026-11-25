#ifndef ROCKO_ENV_IMU_SNAPSHOT_CHECK
#define ROCKO_ENV_IMU_SNAPSHOT_CHECK

// Turns one decoded BNO055 burst (gyro and Euler angles read together, finding H-3) into the values
// the controllers see, or rejects it. Header-only, standard library only, no I/O, no allocation, so
// the decision can be tested on a laptop (tools/imu_snapshot_check_test.cpp) and called from the
// 100 Hz read() of ICM20948.cpp.
//
// Why it is separate from the I2C code: the bus can only be exercised on the robot, but whether a
// sample is believable can be checked anywhere. A sample is all-or-nothing: the angles and the rate
// come from the SAME 12-byte transaction, so if one of them is garbage the whole transaction is
// suspect, and the plugin holds all four last good values (finding C-2: never a made-up 0.0).

#include <cmath>

#include "rocko_env/Bno055UartProtocol.hpp"  // GyroEulerSnapshot; the register decoding is the same on I2C

namespace rocko_env
{

// How the sensor sits on ROCKO. Measured values live in the xacro (hardware parameters), not here.
struct ImuMounting
{
  // [deg] ADDED to the sensor's pitch. The xacro sets 7.6, the value in the old Python IMU node
  // (removed 2026-11-24); its origin is not recorded.
  double pitch_offset_deg = 0.0;
  // Which gyro axis is the pitch rate (0 = X, 1 = Y, 2 = Z) and its sign (+1 or -1), so that
  // pitch_rate = sign * gyro[axis]. Measured 2026-11-23: axis 0 (X), sign -1
  // (docs/journal/2026-11-23-gyro-check.md).
  int pitch_rate_axis = 0;
  int pitch_rate_sign = 1;
};

// What the controllers get from one good sample.
struct ImuReading
{
  double yaw_deg = 0.0;           // [deg] EUL_Heading, as the sensor reports it
  double roll_deg = 0.0;          // [deg] EUL_Roll
  double pitch_deg = 0.0;         // [deg] EUL_Pitch + pitch_offset_deg. Goes DOWN when ROCKO leans forward (measured 2026-11-24)
  double pitch_rate_deg_s = 0.0;  // [deg/s] pitch_rate_sign * gyro[pitch_rate_axis]; + while pitch increases
};

// Plausibility limits. A sample outside them is treated as a failed read. The angle limits are
// DELIBERATELY LOOSE: they catch garbage and NaN, not a wrong-but-plausible angle (the BNO055's own
// ranges are narrower, about +/-90 deg for one of pitch and roll depending on the convention,
// UNVERIFIED which). A real lean that large is the tipover cutoff's job, not this check's.
constexpr double kMaxAbsPitchRollDeg = 180.0;   // [deg] |pitch| (offset included) and |roll|
constexpr double kMaxAbsYawDeg = 360.0;         // [deg] |yaw|; the heading runs 0..360
// [deg/s] The BNO055 gyro's full scale in its fusion modes is +/-2000 deg/s, so a larger rate cannot
// be a real measurement.
constexpr double kMaxAbsPitchRateDegS = 2000.0;

enum class SnapshotVerdict
{
  kOk,
  kNotDecoded,      // the burst was not decoded (wrong length): all NaN
  kAllAnglesEqual,  // heading == roll == pitch: what a starting or reset BNO055 returns (all zeros)
  kAngleOutOfRange, // non-finite, or beyond the limits above
  kRateOutOfRange,  // non-finite, or beyond kMaxAbsPitchRateDegS
  kBadMounting,     // pitch_rate_axis not 0..2 or pitch_rate_sign not +/-1 (a configuration error)
  kPitchJump,       // pitch or its rate jumped more than physics allows since the last good sample (corrupted byte?)
};

// Judge one snapshot. On kOk, `out` holds the new values; on anything else `out` is untouched, so
// the caller keeps the last good ones.
inline SnapshotVerdict check_imu_snapshot(
  const bno055_uart::GyroEulerSnapshot & s, const ImuMounting & m, ImuReading & out)
{
  if (m.pitch_rate_axis < 0 || m.pitch_rate_axis > 2 ||
    (m.pitch_rate_sign != 1 && m.pitch_rate_sign != -1))
  {
    return SnapshotVerdict::kBadMounting;
  }
  if (!s.valid) {
    return SnapshotVerdict::kNotDecoded;
  }
  // Real data has 1/16 deg resolution and three independent angles, so three exactly equal values
  // almost never happen; the all-zero block a starting or reset sensor returns does (ICM20948.py had
  // the same rule; tools/imu_tilt_log.py saw the zeros at start-up). Zeros plus the offset would
  // look like a believable pitch. The gyro is NOT in this rule: a still gyro really reads 0, 0, 0.
  if (s.heading_deg == s.roll_deg && s.roll_deg == s.pitch_deg) {
    return SnapshotVerdict::kAllAnglesEqual;
  }
  const double pitch_deg = s.pitch_deg + m.pitch_offset_deg;
  if (!std::isfinite(s.heading_deg) || !std::isfinite(s.roll_deg) || !std::isfinite(pitch_deg) ||
    std::fabs(pitch_deg) > kMaxAbsPitchRollDeg || std::fabs(s.roll_deg) > kMaxAbsPitchRollDeg ||
    std::fabs(s.heading_deg) > kMaxAbsYawDeg)
  {
    return SnapshotVerdict::kAngleOutOfRange;
  }
  const double gyro_deg_s[3] = {s.gyro_x_deg_s, s.gyro_y_deg_s, s.gyro_z_deg_s};
  const double rate_deg_s = m.pitch_rate_sign * gyro_deg_s[m.pitch_rate_axis];
  if (!std::isfinite(rate_deg_s) || std::fabs(rate_deg_s) > kMaxAbsPitchRateDegS) {
    return SnapshotVerdict::kRateOutOfRange;
  }
  out.yaw_deg = s.heading_deg;
  out.roll_deg = s.roll_deg;
  out.pitch_deg = pitch_deg;
  out.pitch_rate_deg_s = rate_deg_s;
  return SnapshotVerdict::kOk;
}

// ---- The pitch-jump check: a corrupted but in-range byte (finding from the 2026-11-24 realtime audit) ----
//
// I2C has no checksum. A bit flipped in the high byte of the pitch register moves pitch by 16, 32 or
// 64 deg and still passes check_imu_snapshot()'s range limits. But the gyro measured how fast pitch
// was changing, so a real change between two good samples is at most about |rate| x elapsed time.
// A jump much larger than that is a corrupted sample, treated as a failed read.
//
// [deg] Slack on top of |rate| x dt: the fused angle and the gyro disagree a little, and the rate
// changes within a cycle. PROPOSED; check on the robot that no real motion trips it (the log counts
// "pitch jump" rejects).
constexpr double kMaxPitchJumpSlackDeg = 5.0;
// [s] After a gap this long since the last good sample the check is skipped and the new sample
// accepted: the last good pitch is too old to compare with, and the gyro's rate then says nothing
// about the whole gap. 0.1 s = 10 cycles; by then the fault tracker (5 in a row) has already made
// read() return ERROR, so the controllers have stopped.
constexpr double kMaxPitchJumpGapS = 0.1;

// The rate can be corrupted the same way (a flipped high byte moves it by 16 to 1024 deg/s, inside
// the 2000 deg/s limit), so it gets its own step limit: how fast the pitch rate can really change.
// [deg/s^2] PROPOSED, deliberately generous: an inverted pendulum this size falling at 90 deg
// accelerates at very roughly 5000 deg/s^2; 20000 leaves room for a wheel kick. A flip in bit 8 or
// higher of the rate in one 10 ms cycle (>= 256 deg/s against an allowance of 250) is caught; smaller
// corruptions are not.
constexpr double kMaxPitchAccelDegS2 = 20000.0;
constexpr double kMaxRateStepSlackDegS = 50.0;  // [deg/s] slack on top of accel x dt

// True if the new sample is believable after the last good one, `elapsed_s` seconds earlier:
//  - pitch moved at most |last rate| x dt + kMaxPitchJumpSlackDeg (only the LAST, already trusted,
//    rate sets the allowance, so a corrupted new rate cannot widen it);
//  - the rate changed at most kMaxPitchAccelDegS2 x dt + kMaxRateStepSlackDegS.
inline bool pitch_step_plausible(
  double last_pitch_deg, double last_rate_deg_s, double new_pitch_deg, double new_rate_deg_s,
  double elapsed_s)
{
  if (!std::isfinite(last_pitch_deg) || !std::isfinite(last_rate_deg_s) || elapsed_s > kMaxPitchJumpGapS) {
    return true;  // nothing trustworthy to compare with
  }
  const double dt_s = std::fmax(elapsed_s, 0.0);
  const double allowed_deg = std::fabs(last_rate_deg_s) * dt_s + kMaxPitchJumpSlackDeg;
  const double allowed_rate_step_deg_s = kMaxPitchAccelDegS2 * dt_s + kMaxRateStepSlackDegS;
  return std::fabs(new_pitch_deg - last_pitch_deg) <= allowed_deg &&
    std::fabs(new_rate_deg_s - last_rate_deg_s) <= allowed_rate_step_deg_s;
}

// For the throttled log line.
inline const char * to_string(SnapshotVerdict v)
{
  switch (v) {
    case SnapshotVerdict::kOk: return "ok";
    case SnapshotVerdict::kNotDecoded: return "not decoded";
    case SnapshotVerdict::kAllAnglesEqual: return "all three angles equal (sensor starting or reset?)";
    case SnapshotVerdict::kAngleOutOfRange: return "angle out of range";
    case SnapshotVerdict::kRateOutOfRange: return "pitch rate out of range";
    case SnapshotVerdict::kBadMounting: return "bad pitch_rate_axis or pitch_rate_sign";
    case SnapshotVerdict::kPitchJump: return "pitch or pitch-rate jump too large (corrupted sample?)";
  }
  return "unknown";
}

}  // namespace rocko_env

#endif  // ROCKO_ENV_IMU_SNAPSHOT_CHECK
