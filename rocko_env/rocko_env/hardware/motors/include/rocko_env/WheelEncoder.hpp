#ifndef ROCKO_ENV_WHEEL_ENCODER
#define ROCKO_ENV_WHEEL_ENCODER

// The arithmetic of one wheel's quadrature encoder, kept apart from the GPIO code so it can be
// tested on a laptop (tools/wheel_encoder_test.cpp). Standard library only, no I/O, no allocation.
//
// Since 2026-11-24 (finding C-1) the motor plugin counts encoder edges itself; before, it asked a
// Python node (QuadEncoders.py) over a ROS service and waited with no time limit. Both pieces here
// copy that node on purpose, so the controllers see the SAME signal as before (the gains and the
// sim's encoder replica were tuned against it):
//   - quadrature_step() is the table of the Python `Encoder` library the node used (a port of Paul
//     Stoffregen's Arduino Encoder library): 4 counts per full A/B cycle;
//   - WheelVelocityFilter is the node's WheelChannel.update(): travel since the last cycle divided by
//     the real time it took, then low-passed.

#include <cmath>

namespace rocko_env
{

// A quadrature encoder has two outputs, A and B, square waves a quarter cycle apart. The count goes
// UP when B changes first: (A, B) = (0,0) -> (0,1) -> (1,1) -> (1,0) -> (0,0), which is states
// 0 -> 2 -> 3 -> 1 -> 0 below (bit 0 = A, bit 1 = B). The other way round it goes down. Each change
// is one count, so one full A/B cycle is 4 counts. Which way is "forward" for a wheel is set by
// which wire is called A in the xacro (the right wheel's are swapped).
inline int quadrature_state(bool a_high, bool b_high)
{
  return (a_high ? 1 : 0) | (b_high ? 2 : 0);
}

// Counts moved going from `previous_state` to `new_state` (each 0..3, from quadrature_state()).
//   +1: 0->2, 2->3, 3->1, 1->0 (one step up the sequence above)
//   -1: 0->1, 1->3, 3->2, 2->0 (one step down)
//   +2: 0->3, 3->0;  -2: 1->2, 2->1 (both bits changed: a state was missed because the pins moved
//       faster than they were read; the direction is a guess, the table the Python Encoder library
//       used, which the speed signal was tuned with)
//    0: no change
// The switch key is previous + 4 x new, so 0->2 is 0 + 4*2 = 8, 2->3 is 14, 3->1 is 7, 1->0 is 1.
inline int quadrature_step(int previous_state, int new_state)
{
  switch ((previous_state & 3) + 4 * (new_state & 3)) {
    case 1: case 7: case 8: case 14:   // 1->0, 3->1, 0->2, 2->3
      return +1;
    case 2: case 4: case 11: case 13:  // 2->0, 0->1, 3->2, 1->3
      return -1;
    case 3: case 12:                   // 3->0, 0->3
      return +2;
    case 6: case 9:                    // 2->1, 1->2
      return -2;
    default:
      return 0;  // no change (0, 5, 10, 15)
  }
}

// [counts per metre of wheel travel] from the counts in one wheel turn and the wheel's radius [m].
inline double counts_per_metre(double counts_per_wheel_rev, double wheel_radius_m)
{
  return counts_per_wheel_rev / (2.0 * M_PI * wheel_radius_m);
}

// Low-pass filter weight of the newest velocity sample: filtered = alpha * new + (1 - alpha) * filtered.
// 0.05, the value QuadEncoders.py used. It is applied once per control cycle, so its time constant
// is about cycle / alpha: ~0.2 s at 100 Hz (balancing), ~0.4 s at 50 Hz (tank). Changing it changes
// the velocity loop's lag, so it is a tuning decision, not a tidy-up.
constexpr double kWheelVelocityFilterAlpha = 0.05;

// One wheel's velocity [m/s] from its position [m], once per control cycle.
struct WheelVelocityFilter
{
  bool has_reference = false;
  double reference_position_m = 0.0;  // position at the last cycle that produced a velocity
  double elapsed_s = 0.0;             // time since that cycle
  double velocity_m_s = 0.0;          // the filtered velocity, what the controllers see

  // Forget the history: the next update() only takes a reference and reports 0.0.
  void reset()
  {
    has_reference = false;
    elapsed_s = 0.0;
    velocity_m_s = 0.0;
  }

  // `period_s` is the time since the previous update(), measured by the caller when it took the
  // position (the plugin uses steady_clock in read()).
  double update(double position_m, double period_s)
  {
    if (!has_reference) {
      // First cycle: nothing to compare with, so no velocity yet; report 0.0 (not seen moving).
      reference_position_m = position_m;
      elapsed_s = 0.0;
      has_reference = true;
      return velocity_m_s;
    }
    // A period that is not a number (it should never be, but it comes from outside this code) is
    // skipped rather than allowed to poison every later velocity.
    if (std::isfinite(period_s)) {
      elapsed_s += period_s;
    }
    if (!(elapsed_s > 0.0)) {
      // No time has passed: dividing by it is impossible, so keep the last velocity AND the old
      // reference. Example: cycle 1 has period 0, so cycle 2 measures the travel of both cycles over
      // the time of both. Without that, the travel during cycle 1 would never show up as speed.
      return velocity_m_s;
    }
    const double raw_velocity_m_s = (position_m - reference_position_m) / elapsed_s;
    velocity_m_s = kWheelVelocityFilterAlpha * raw_velocity_m_s +
      (1.0 - kWheelVelocityFilterAlpha) * velocity_m_s;
    reference_position_m = position_m;
    elapsed_s = 0.0;
    return velocity_m_s;
  }
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_WHEEL_ENCODER
