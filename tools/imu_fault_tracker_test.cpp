// Host-side self-test for rocko_env::ImuFaultTracker. No ROS needed; it runs on a laptop.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
//       tools/imu_fault_tracker_test.cpp -o /tmp/imu_fault_tracker_test && /tmp/imu_fault_tracker_test
// Exit status 0 and the line "all ImuFaultTracker tests passed" mean success. A failed check
// prints the line and aborts (non-zero). This proves the counting logic only; it says nothing
// about the sensor, the plugin, or the robot.

#undef NDEBUG  // asserts must stay on even if someone adds -DNDEBUG
#include <cassert>
#include <cstdio>

#include "rocko_env/ImuFaultTracker.hpp"

using rocko_env::ImuFaultTracker;

namespace
{
constexpr int kMaxConsecutive = 5;  // [reads] same value the plugin uses

void feed(ImuFaultTracker & t, bool good, int times)
{
  for (int i = 0; i < times; ++i) {
    t.record(good);
  }
}

void test_consecutive_trip_at_threshold()
{
  ImuFaultTracker t(kMaxConsecutive);
  feed(t, false, kMaxConsecutive - 1);
  assert(!t.tripped());
  t.record(false);
  assert(t.tripped());
  assert(t.consecutive_failures() == kMaxConsecutive);
}

void test_good_read_clears_consecutive_run()
{
  ImuFaultTracker t(kMaxConsecutive);
  feed(t, false, kMaxConsecutive);
  assert(t.tripped());
  t.record(true);
  assert(t.consecutive_failures() == 0);
  assert(!t.tripped());  // window holds only 5 failures, below its limit of 10
  feed(t, false, kMaxConsecutive - 1);
  assert(!t.tripped());  // the run restarted from zero
}

void test_alternating_failures_trip_via_window()
{
  ImuFaultTracker t(kMaxConsecutive);
  bool ever_tripped = false;
  int max_run = 0;
  for (int cycle = 0; cycle < 10; ++cycle) {  // 10 x (4 bad + 1 good) = 50 reads
    feed(t, false, 4);
    if (t.consecutive_failures() > max_run) {
      max_run = t.consecutive_failures();
    }
    ever_tripped = ever_tripped || t.tripped();
    t.record(true);
    ever_tripped = ever_tripped || t.tripped();
  }
  assert(max_run == 4);        // the consecutive limit really was never reached
  assert(ever_tripped);        // so the window is what caught it
  assert(t.tripped());         // and it is still tripped at steady state (16 of 20 failed)
  assert(t.window_failures() == 16);
}

void test_window_trip_clears_after_good_reads()
{
  ImuFaultTracker t(kMaxConsecutive);
  for (int cycle = 0; cycle < 10; ++cycle) {
    feed(t, false, 4);
    t.record(true);
  }
  assert(t.tripped());
  feed(t, true, ImuFaultTracker::kWindowReads);  // a full window of good reads flushes the ring
  assert(t.window_failures() == 0);
  assert(!t.tripped());
}

void test_total_failures_keeps_counting()
{
  ImuFaultTracker t(kMaxConsecutive);
  feed(t, false, 3);
  t.record(true);
  feed(t, false, 2);
  assert(t.total_failures() == 5);
  feed(t, false, 100);  // long outage: consecutive count saturates, total does not
  assert(t.consecutive_failures() == kMaxConsecutive);
  assert(t.total_failures() == 105);
}

void test_isolated_failures_never_trip()
{
  ImuFaultTracker t(kMaxConsecutive);
  for (int i = 0; i < 500; ++i) {
    t.record(i % 10 != 0);  // one bad read in every 10
    assert(!t.tripped());
  }
  assert(t.total_failures() == 50);
}

void test_custom_window_threshold()
{
  ImuFaultTracker t(kMaxConsecutive, 3);  // trips after 3 failures within the window
  t.record(false);
  t.record(true);
  t.record(false);
  assert(!t.tripped());
  t.record(false);
  assert(t.tripped());
}
}  // namespace

int main()
{
  test_consecutive_trip_at_threshold();
  test_good_read_clears_consecutive_run();
  test_alternating_failures_trip_via_window();
  test_window_trip_clears_after_good_reads();
  test_total_failures_keeps_counting();
  test_isolated_failures_never_trip();
  test_custom_window_threshold();
  std::printf("all ImuFaultTracker tests passed\n");
  return 0;
}
