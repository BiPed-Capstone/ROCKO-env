#ifndef ROCKO_ENV_IMU_FAULT_TRACKER
#define ROCKO_ENV_IMU_FAULT_TRACKER

#include <array>

namespace rocko_env
{

// Counts failed IMU reads and says when there have been too many (finding C-2; the findings are
// described in docs/claude-analysis/balance-loop-findings.md). This is the IMU fault detector.
//
// Why it exists: a balancing robot cannot be blind for long. One missed sample can be covered by
// holding the last good angle, but a held angle gets staler every cycle. Two things trip it:
//   1. Consecutive failures: max_consecutive_failures failed reads in a row.
//   2. Windowed failures: max_window_failures failed reads among the last kWindowReads reads.
//      This catches a sensor that alternates (for example 4 bad reads, 1 good, repeat), which
//      never reaches the consecutive limit but is still mostly blind.
//
// What the caller does with it: ICM20948::read() turns tripped() into a return code and returns
// ERROR. What ros2_control does after read() returns ERROR (whether it keeps calling read(), and what
// happens to the other components) has not been observed here. Nothing in this class stops a controller, and the class itself is not
// latched: tripped() goes back to false as soon as the counts fall below the limits.
//
// Hysteresis (how it un-trips): a good read clears the consecutive count immediately. The windowed
// count falls only as the ring fills with good reads: each good read overwrites the oldest entry,
// so after a burst of failures the windowed trip clears once enough old failures have been pushed
// out of the last kWindowReads reads. tripped() therefore returns to false on the first good read
// only if the windowed count is also below its limit; otherwise it stays true until it is.
// The fault is not latched: there is no reset call and no "stay stopped" state in this class.
//
// Safe to call from read() at 100 Hz: a fixed std::array and plain integers. No allocation, no
// logging, no locking, no exceptions, and no ROS types, so a simulator plugin is planned to reuse
// it.
//
// Not thread-safe: call record() from one thread (the one running read()).
class ImuFaultTracker
{
public:
  // Size of the sliding window, in reads. 20 reads = 200 ms at 100 Hz.
  static constexpr int kWindowReads = 20;  // [reads]

  // SAFETY PARAMETER: proposed value, a human confirms on the robot.
  // Failed reads within the last kWindowReads that make tripped() true. 10 of 20 = half blind.
  static constexpr int kMaxWindowFailures = 10;  // [reads]

  // max_consecutive_failures: failed reads in a row that make tripped() true.
  //   One read per control cycle, so at 100 Hz this is also the number of 10 ms cycles.
  // max_window_failures: failed reads among the last kWindowReads that make tripped() true.
  explicit ImuFaultTracker(
    int max_consecutive_failures, int max_window_failures = kMaxWindowFailures)
  : max_consecutive_failures_(max_consecutive_failures),
    max_window_failures_(max_window_failures) {}

  // Call once per read cycle. good_read is true only if the sample can be used.
  void record(bool good_read)
  {
    // The oldest entry in the ring is about to be replaced; take it out of the running count.
    // The ring starts all-false (= "good"), so the first kWindowReads reads behave correctly.
    if (window_failed_[next_slot_]) {
      --window_failures_;
    }
    window_failed_[next_slot_] = !good_read;
    if (!good_read) {
      ++window_failures_;
    }
    next_slot_ = (next_slot_ + 1) % kWindowReads;

    if (good_read) {
      // Only the run of failures is cleared. total_failures_ keeps counting so a sensor that
      // fails now and then is still visible after the fact.
      consecutive_failures_ = 0;
    } else {
      // Stop counting at the threshold so a long outage cannot overflow the int.
      if (consecutive_failures_ < max_consecutive_failures_) {
        ++consecutive_failures_;
      }
      ++total_failures_;
    }
  }

  // True while either limit is reached (see "Hysteresis" above). Not latched.
  bool tripped() const
  {
    return consecutive_failures_ >= max_consecutive_failures_ ||
           window_failures_ >= max_window_failures_;
  }

  // Failed reads in a row right now (0 after any good read).
  int consecutive_failures() const { return consecutive_failures_; }

  // Failed reads among the last kWindowReads reads.
  int window_failures() const { return window_failures_; }

  // Failed reads since construction, never reset.
  unsigned long total_failures() const { return total_failures_; }

private:
  int max_consecutive_failures_;      // [reads] consecutive threshold for tripped()
  int max_window_failures_;           // [reads] windowed threshold for tripped()
  int consecutive_failures_ = 0;      // [reads] current run of failures
  int window_failures_ = 0;           // [reads] failures among the last kWindowReads reads
  int next_slot_ = 0;                 // [index] ring slot the next read overwrites
  std::array<bool, kWindowReads> window_failed_{};  // true = that read failed; starts all false
  unsigned long total_failures_ = 0;  // [reads] all failures since construction
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_IMU_FAULT_TRACKER
