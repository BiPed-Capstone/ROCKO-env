#ifndef ROCKO_ENV_FEEDFORWARD_TIMEOUT
#define ROCKO_ENV_FEEDFORWARD_TIMEOUT

#include <atomic>
#include <cstdint>
#include <limits>

namespace rocko_env
{

// The age limit on AddFeedforwardController's feedforward (finding H-11, found in stage 4 round 3).
// No ROS types, so a host unit test can check it: tools/feedforward_timeout_test.cpp.
//
// Why it exists: DiffDriveController.py publishes each wheel's feedforward [rad/s] every 10 ms.
// The controller adds the LAST value it received to the wheel command. If that node dies in the
// middle of a turn, nothing ever replaces the last value, so the two wheels keep opposite biases
// and the gamepad cannot stop them. C-3's teleop watchdog cannot help: it lives inside the node
// that died. So the feedforward carries the time it arrived, and once it is older than the limit
// the controller adds 0.0 instead.
//
// Only the FEEDFORWARD term goes to 0.0. The reference from pitch_pid_controller still goes through,
// so the balance loop keeps holding the robot up. That is C-3's rule too: while the robot is upright,
// zeroing the wheel output would drop it (TipoverLatch.hpp explains when zero output IS right).
//
// The rules, all in update():
//   - Nothing has arrived yet           -> 0.0, and it is not "timed out" (what the controller did
//                                          before this existed: the feedforward started at 0.0).
//   - Arrived, age <= max_age           -> the value.
//   - Arrived, age >  max_age           -> 0.0, timed out. just_timed_out() is true for the one
//                                          update() where that starts, so the caller logs once.
//   - A fresh message after a time-out  -> the value again, at once.
//   - age < 0 (the message's stamp is a little later than the `now` the loop read, because it
//     arrived in between) -> treated as age 0. The clock must be MONOTONIC (steady), so this is the
//     only way age can be negative. With a clock that can jump back (system time, set by NTP) a
//     negative age could last for as long as the jump and keep a dead feedforward alive.
//
// Threads: store() is called from the subscription callback (executor thread); everything else from
// the control loop only. Both sides use only lock-free atomics: no allocation, no locks, no logging.
class FeedforwardTimeout
{
public:
  // Any thread. value: the feedforward [rad/s], already checked finite by the caller.
  // now_ns: when it arrived [ns], on the same monotonic clock the loop passes to update().
  void store(double value, std::int64_t now_ns)
  {
    // Value first, stamp second (both sequentially consistent). A reader that sees the new stamp
    // therefore also sees the new value. A reader that sees the old stamp judges by the old one,
    // which can only make it a cycle late in noticing the new message, never use a value as fresh
    // that is not.
    value_.store(value);
    stamp_ns_.store(now_ns);
  }

  // Control loop, once per cycle. Returns the feedforward to add this cycle [rad/s].
  // now_ns: this cycle's time [ns], same clock as store(). max_age_ns: the limit [ns].
  double update(std::int64_t now_ns, std::int64_t max_age_ns)
  {
    const std::int64_t stamp_ns = stamp_ns_.load();
    const double value = value_.load();
    bool fresh = false;
    if (stamp_ns != kNever) {
      age_ns_ = now_ns > stamp_ns ? now_ns - stamp_ns : 0;  // negative age counts as 0, see above
      fresh = age_ns_ <= max_age_ns;
    } else {
      age_ns_ = 0;
    }
    const bool timed_out_now = stamp_ns != kNever && !fresh;
    just_timed_out_ = timed_out_now && !timed_out_;
    timed_out_ = timed_out_now;
    return fresh ? value : 0.0;
  }

  // True only in the update() call where a time-out starts (a feedforward has arrived and has just
  // become older than the limit): log then, not every cycle.
  bool just_timed_out() const { return just_timed_out_; }
  // [ns] age of the last feedforward at the last update(); 0 if none has arrived.
  std::int64_t age_ns() const { return age_ns_; }

private:
  static constexpr std::int64_t kNever = std::numeric_limits<std::int64_t>::min();
  std::atomic<double> value_{0.0};                 // [rad/s] last finite feedforward received
  std::atomic<std::int64_t> stamp_ns_{kNever};     // [ns] when it arrived; kNever = not yet
  static_assert(
    std::atomic<double>::is_always_lock_free && std::atomic<std::int64_t>::is_always_lock_free,
    "the feedforward and its stamp are read in the real-time loop and must not take a lock");
  // Control loop only.
  bool timed_out_ = false;
  bool just_timed_out_ = false;
  std::int64_t age_ns_ = 0;                        // [ns]
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_FEEDFORWARD_TIMEOUT
