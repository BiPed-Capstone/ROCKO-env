// Host-side self-test for rocko_env::FeedforwardTimeout (finding H-11). No ROS needed; it runs on a
// laptop.
//
// Build and run (from the repo root):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include \
//       tools/feedforward_timeout_test.cpp -o /tmp/feedforward_timeout_test && /tmp/feedforward_timeout_test
// Exit status 0 and the line "all FeedforwardTimeout tests passed" mean success. A failed check
// prints the line and aborts (non-zero). This proves the age decision only; it says nothing about
// the controller wiring, DDS delivery, the clock on the Pi, or the robot.

#undef NDEBUG  // asserts must stay on even if someone adds -DNDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>

#include "rocko_env/FeedforwardTimeout.hpp"

using rocko_env::FeedforwardTimeout;

namespace
{
constexpr std::int64_t kMs = 1'000'000;          // [ns] one millisecond
constexpr std::int64_t kMaxAgeNs = 200 * kMs;    // [ns] the proposed kMaxFeedforwardAgeNs
constexpr std::int64_t kStartNs = 5'000 * kMs;   // [ns] an arbitrary steady-clock time

void test_nothing_arrived_is_zero_and_not_timed_out()
{
  FeedforwardTimeout f;
  for (int cycle = 0; cycle < 1000; ++cycle) {  // 10 s of cycles with no message at all
    assert(f.update(kStartNs + cycle * 10 * kMs, kMaxAgeNs) == 0.0);
    assert(!f.just_timed_out());
  }
}

void test_fresh_value_is_used_up_to_the_limit()
{
  FeedforwardTimeout f;
  f.store(5.0, kStartNs);
  assert(f.update(kStartNs, kMaxAgeNs) == 5.0);
  assert(f.update(kStartNs + 100 * kMs, kMaxAgeNs) == 5.0);
  assert(f.update(kStartNs + kMaxAgeNs, kMaxAgeNs) == 5.0);  // exactly at the limit: still fresh
  assert(!f.just_timed_out());
  assert(f.age_ns() == kMaxAgeNs);
}

void test_old_value_becomes_zero_and_logs_once()
{
  FeedforwardTimeout f;
  f.store(-3.0, kStartNs);
  assert(f.update(kStartNs + kMaxAgeNs + 1, kMaxAgeNs) == 0.0);  // 1 ns past the limit
  assert(f.just_timed_out());
  for (int cycle = 1; cycle <= 100; ++cycle) {  // stays 0.0, but "just" only the first time
    assert(f.update(kStartNs + kMaxAgeNs + cycle * 10 * kMs, kMaxAgeNs) == 0.0);
    assert(!f.just_timed_out());
  }
}

void test_fresh_message_after_time_out_is_used_at_once()
{
  FeedforwardTimeout f;
  f.store(2.0, kStartNs);
  const std::int64_t later = kStartNs + 1000 * kMs;
  assert(f.update(later, kMaxAgeNs) == 0.0);
  assert(f.just_timed_out());
  f.store(4.0, later + 5 * kMs);
  assert(f.update(later + 10 * kMs, kMaxAgeNs) == 4.0);
  assert(!f.just_timed_out());
  // A second silence is a second time-out, and is reported again.
  assert(f.update(later + 10 * kMs + 300 * kMs, kMaxAgeNs) == 0.0);
  assert(f.just_timed_out());
}

void test_steady_stream_never_times_out()
{
  // DiffDriveController.py publishes every 10 ms; with a 60 ms delivery hiccup now and then the
  // value must never be dropped.
  FeedforwardTimeout f;
  std::int64_t t = kStartNs;
  for (int msg = 0; msg < 500; ++msg) {
    t += (msg % 50 == 49) ? 60 * kMs : 10 * kMs;
    f.store(1.5, t);
    assert(f.update(t + 5 * kMs, kMaxAgeNs) == 1.5);
    assert(!f.just_timed_out());
  }
}

void test_stamp_slightly_after_now_counts_as_fresh()
{
  // The loop reads `now`, then the message arrives and is stamped a little later, then the loop
  // reads the stamp: the age comes out negative. That is a brand-new message, not a stale one.
  FeedforwardTimeout f;
  f.store(7.0, kStartNs + 2 * kMs);
  assert(f.update(kStartNs, kMaxAgeNs) == 7.0);
  assert(!f.just_timed_out());
  assert(f.age_ns() == 0);
}

void test_zero_value_is_a_real_value()
{
  // 0.0 is what the node sends when standing still: fresh, and not a time-out.
  FeedforwardTimeout f;
  f.store(0.0, kStartNs);
  assert(f.update(kStartNs + 10 * kMs, kMaxAgeNs) == 0.0);
  assert(!f.just_timed_out());
}
}  // namespace

int main()
{
  test_nothing_arrived_is_zero_and_not_timed_out();
  test_fresh_value_is_used_up_to_the_limit();
  test_old_value_becomes_zero_and_logs_once();
  test_fresh_message_after_time_out_is_used_at_once();
  test_steady_stream_never_times_out();
  test_stamp_slightly_after_now_counts_as_fresh();
  test_zero_value_is_a_real_value();
  std::puts("all FeedforwardTimeout tests passed");
  return 0;
}
