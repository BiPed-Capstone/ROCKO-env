// Host-side self-test for rocko_env::TipoverLatch (finding H-4). No ROS needed; it runs on a laptop.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include \
//       tools/tipover_latch_test.cpp -o /tmp/tipover_latch_test && /tmp/tipover_latch_test
// Exit status 0 and the line "all TipoverLatch tests passed" mean success. A failed check prints
// the line and aborts (non-zero). This proves the trip/re-arm decision only; it says nothing about
// the controller wiring, the IMU, the motors, or the robot.

#undef NDEBUG  // asserts must stay on even if someone adds -DNDEBUG
#include <cassert>
#include <cstdio>
#include <limits>

#include "rocko_env/TipoverLatch.hpp"

using rocko_env::TipoverLatch;
using Event = rocko_env::TipoverLatch::Event;

namespace
{
constexpr double kLimitDeg = 35.0;     // [deg] the proposed default of tipover_pitch_deg
constexpr double kRearmMaxDeg = 10.0;  // [deg] the proposed default of rearm_max_pitch_deg
const double kNaN = std::numeric_limits<double>::quiet_NaN();

void test_trips_above_limit_not_at_it()
{
  TipoverLatch t;
  assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);
  assert(t.update(35.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);  // exactly at the limit: still armed
  assert(t.armed());
  assert(t.update(35.1, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  assert(!t.armed());
  assert(t.trips() == 1);
}

void test_trips_leaning_backward_too()
{
  TipoverLatch t;
  assert(t.update(-40.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  assert(!t.armed());
}

void test_nan_does_not_trip()
{
  TipoverLatch t;
  for (int i = 0; i < 100; ++i) {
    assert(t.update(kNaN, kLimitDeg, kRearmMaxDeg) == Event::kNone);
  }
  assert(t.armed());
  assert(t.trips() == 0);
}

void test_stays_disarmed_after_pitch_returns()
{
  TipoverLatch t;
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  for (int i = 0; i < 100; ++i) {
    assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);  // picked up and held upright
  }
  assert(!t.armed());
  assert(t.trips() == 1);  // one trip, not one per cycle
}

void test_rearm_refused_while_past_limit()
{
  TipoverLatch t;
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  t.request_rearm();
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);
  assert(!t.armed());
  // The refused request was used up: coming back inside does not re-arm by itself.
  assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);
  assert(!t.armed());
}

void test_rearm_refused_on_nan_pitch()
{
  TipoverLatch t;
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  t.request_rearm();
  assert(t.update(kNaN, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);
  assert(!t.armed());
}

void test_rearm_accepted_when_inside()
{
  TipoverLatch t;
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  t.request_rearm();
  assert(t.update(2.0, kLimitDeg, kRearmMaxDeg) == Event::kRearmed);
  assert(t.armed());
  // It can trip again, and the count goes on.
  assert(t.update(-36.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  assert(t.trips() == 2);
}

// Hysteresis: once tripped, being back inside the trip limit is not enough to re-arm; the robot
// must be within rearm_max of upright. Each refused request is used up, so each try is a new one.
void test_rearm_needs_pitch_inside_rearm_band()
{
  TipoverLatch t;
  assert(t.update(35.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);     // at the limit: armed
  assert(t.update(35.01, kLimitDeg, kRearmMaxDeg) == Event::kTripped);  // past it: trip
  t.request_rearm();
  assert(t.update(34.9, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);  // inside the trip limit
  t.request_rearm();
  assert(t.update(11.0, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);  // still outside 10 deg
  t.request_rearm();
  assert(t.update(-11.0, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);  // backward too
  t.request_rearm();
  assert(t.update(kNaN, kLimitDeg, kRearmMaxDeg) == Event::kRearmRefused);   // unknown pitch
  assert(!t.armed());
  t.request_rearm();
  assert(t.update(9.9, kLimitDeg, kRearmMaxDeg) == Event::kRearmed);
  assert(t.armed());
}

void test_rearm_max_validity()
{
  assert(TipoverLatch::is_valid_rearm_max(kRearmMaxDeg, kLimitDeg));
  assert(!TipoverLatch::is_valid_rearm_max(0.0, kLimitDeg));        // must be above 0
  assert(!TipoverLatch::is_valid_rearm_max(-5.0, kLimitDeg));
  assert(!TipoverLatch::is_valid_rearm_max(kLimitDeg, kLimitDeg));  // must be below the limit
  assert(!TipoverLatch::is_valid_rearm_max(40.0, kLimitDeg));
  assert(!TipoverLatch::is_valid_rearm_max(kNaN, kLimitDeg));
}

void test_request_while_armed_is_not_kept()
{
  TipoverLatch t;
  t.request_rearm();                                   // sent while nothing had tripped
  assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);    // used up here
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);    // must NOT re-arm from the old request
  assert(!t.armed());
}

void test_disabled_when_limit_not_positive()
{
  const double disabled_limits[] = {0.0, -1.0, kNaN};
  for (double limit : disabled_limits) {
    assert(!TipoverLatch::is_enabled(limit));
    TipoverLatch t;
    assert(t.update(89.0, limit, kRearmMaxDeg) == Event::kNone);
    assert(t.armed());
    assert(t.trips() == 0);
  }
  assert(TipoverLatch::is_enabled(kLimitDeg));
}

void test_reset_rearms_and_drops_request()
{
  TipoverLatch t;
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  t.reset();
  assert(t.armed());
  assert(t.trips() == 1);  // the history is kept
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);  // still fallen: trips again at once
  t.request_rearm();
  t.reset();
  assert(t.update(50.0, kLimitDeg, kRearmMaxDeg) == Event::kTripped);
  assert(t.update(0.0, kLimitDeg, kRearmMaxDeg) == Event::kNone);  // the request did not survive reset()
  assert(!t.armed());
}
}  // namespace

int main()
{
  test_trips_above_limit_not_at_it();
  test_trips_leaning_backward_too();
  test_nan_does_not_trip();
  test_stays_disarmed_after_pitch_returns();
  test_rearm_refused_while_past_limit();
  test_rearm_refused_on_nan_pitch();
  test_rearm_accepted_when_inside();
  test_rearm_needs_pitch_inside_rearm_band();
  test_rearm_max_validity();
  test_request_while_armed_is_not_kept();
  test_disabled_when_limit_not_positive();
  test_reset_rearms_and_drops_request();
  std::puts("all TipoverLatch tests passed");
  return 0;
}
