// Host-side self-test for WheelEncoder.hpp: the quadrature decoding and the velocity filter the motor
// plugin (Motor12VoltQuadEncoder) uses since the C-1 encoder fix (2026-11-24). No ROS, no WiringPi.
// Both pieces must behave exactly like the Python node they replaced (QuadEncoders.py and the Python
// `Encoder` library), so the controllers see the same signal; these tests are the old
// tools/quad_encoders_test.py's velocity cases, ported.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/motors/include \
//       tools/wheel_encoder_test.cpp -o /tmp/wheel_encoder_test && /tmp/wheel_encoder_test
// Exit status 0 and "all WheelEncoder tests passed" mean success. This proves the arithmetic only:
// the GPIO interrupts, the pin numbers and the threads are checked on the robot, not here.

#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>

#include "rocko_env/WheelEncoder.hpp"

using rocko_env::quadrature_state;
using rocko_env::quadrature_step;
using rocko_env::WheelVelocityFilter;

namespace
{
bool close(double a, double b, double rel = 1e-9)
{
  return std::fabs(a - b) <= rel * std::fmax(std::fabs(a), std::fabs(b));
}

// The Python Encoder library's update(), written out as it is there, to compare every transition.
int python_library_step(int previous_state, bool a, bool b)
{
  int state = previous_state & 3;
  if (a) {state |= 4;}
  if (b) {state |= 8;}
  if (state == 1 || state == 7 || state == 8 || state == 14) {return 1;}
  if (state == 2 || state == 4 || state == 11 || state == 13) {return -1;}
  if (state == 3 || state == 12) {return 2;}
  if (state == 6 || state == 9) {return -2;}
  return 0;
}

void test_table_matches_the_python_library()
{
  for (int previous = 0; previous < 4; ++previous) {
    for (int a = 0; a < 2; ++a) {
      for (int b = 0; b < 2; ++b) {
        assert(quadrature_step(previous, quadrature_state(a, b)) == python_library_step(previous, a, b));
      }
    }
  }
}

// Counting up is B leading A: (A,B) = (0,0), (0,1), (1,1), (1,0) (bit 0 = A, so states 0, 2, 3, 1).
const bool kForwardA[4] = {false, false, true, true};
const bool kForwardB[4] = {false, true, true, false};

long run_cycles(int cycles, bool forward, bool swap_pins)
{
  long count = 0;
  int state = quadrature_state(false, false);
  for (int i = 1; i <= 4 * cycles; ++i) {
    const int k = forward ? (i % 4) : ((4 - i % 4) % 4);
    bool a = kForwardA[k];
    bool b = kForwardB[k];
    if (swap_pins) {
      const bool t = a; a = b; b = t;
    }
    const int next = quadrature_state(a, b);
    count += quadrature_step(state, next);
    state = next;
  }
  return count;
}

void test_four_counts_per_cycle_and_direction()
{
  assert(run_cycles(10, true, false) == 40);
  assert(run_cycles(10, false, false) == -40);
  // Swapping A and B reverses the sign: this is why the right wheel's pins are swapped in the xacro
  // (its encoder turns the other way for forward travel).
  assert(run_cycles(10, true, true) == -40);
}

void test_no_change_is_zero()
{
  for (int s = 0; s < 4; ++s) {
    assert(quadrature_step(s, s) == 0);
  }
}

void test_scale_matches_the_python_node()
{
  // QuadEncoders.py: COUNTS_PER_METER = 145 * 1.355 / (2 * 0.06 * pi); the xacro gives 196.475 and 0.06.
  const double python = 145 * 1.355 / (2 * 0.06 * M_PI);
  assert(close(rocko_env::counts_per_metre(196.475, 0.06), python));
}

// Feed `cycles` cycles of a constant speed (`counts_per_cycle` every `period_s`) and return the velocity.
double settle(double counts_per_cycle, double period_s, int cycles = 400)
{
  const double cpm = rocko_env::counts_per_metre(196.475, 0.06);
  WheelVelocityFilter f;
  double counts = 0.0;
  double v = f.update(0.0, period_s);
  for (int i = 0; i < cycles; ++i) {
    counts += counts_per_cycle;
    v = f.update(counts / cpm, period_s);
  }
  return v;
}

void test_first_update_reports_zero()
{
  WheelVelocityFilter f;
  assert(f.update(5.0, 0.01) == 0.0);  // already moved before the first cycle: no history, no speed
}

void test_steady_speed_is_read_at_full_value()
{
  const double cpm = rocko_env::counts_per_metre(196.475, 0.06);
  const double expected_m_s = 10 / cpm / 0.01;
  assert(close(settle(10, 0.01), expected_m_s, 1e-3));
}

void test_velocity_uses_the_period()
{
  // One speed reads the same at 10 ms and 20 ms cycles; the same counts in twice the time is half.
  const double fast = settle(10, 0.01);
  assert(close(fast, settle(20, 0.02), 1e-3));
  assert(close(settle(10, 0.02), fast / 2, 1e-3));
}

void test_filter_time_constant()
{
  // A step to a constant speed: after one cycle the output is alpha of it (0.05), as in the Python node.
  WheelVelocityFilter f;
  f.update(0.0, 0.01);
  const double v = f.update(0.01, 0.01);  // 1 m/s for one cycle
  assert(close(v, rocko_env::kWheelVelocityFilterAlpha * 1.0));
}

void test_no_time_elapsed_is_safe()
{
  WheelVelocityFilter f;
  f.update(0.0, 0.01);
  const double before = f.update(0.1, 0.01);
  const double same_time = f.update(0.2, 0.0);       // no time passed
  assert(std::isfinite(same_time) && same_time == before);
  const double backwards = f.update(0.3, -0.01);     // a negative period: never divide by it
  assert(std::isfinite(backwards) && backwards == before);
  const double not_a_number = f.update(0.3, NAN);   // a NaN period is ignored
  assert(std::isfinite(not_a_number) && not_a_number == before);
  // The reference was kept, so the next good cycle sees all the motion since it (0.1 -> 0.3 m).
  // Elapsed: 0.0 - 0.01 (ignored NaN) + 0.02 = 0.01 s, the same total time the Python node measured.
  const double after = f.update(0.3, 0.02);
  assert(after > before);
  assert(close(after, (1 - 0.05) * before + 0.05 * (0.2 / 0.01)));
}

void test_reset_forgets_history()
{
  WheelVelocityFilter f;
  f.update(0.0, 0.01);
  f.update(1.0, 0.01);
  f.reset();
  assert(f.velocity_m_s == 0.0);
  assert(f.update(50.0, 0.01) == 0.0);  // a jump in position after reset is not a speed
}
}  // namespace

int main()
{
  test_table_matches_the_python_library();
  test_four_counts_per_cycle_and_direction();
  test_no_change_is_zero();
  test_scale_matches_the_python_node();
  test_first_update_reports_zero();
  test_steady_speed_is_read_at_full_value();
  test_velocity_uses_the_period();
  test_filter_time_constant();
  test_no_time_elapsed_is_safe();
  test_reset_forgets_history();
  std::puts("all WheelEncoder tests passed");
  return 0;
}
