#ifndef ROCKO_ENV_TIPOVER_LATCH
#define ROCKO_ENV_TIPOVER_LATCH

#include <atomic>
#include <cmath>

namespace rocko_env
{

// The decision part of the tipover cutoff (finding H-4; the findings are described in
// docs/claude-analysis/balance-loop-findings.md). No ROS types, so a host unit test can check it:
// tools/tipover_latch_test.cpp.
//
// Why it exists (physics, not appearances): past the lean from which the robot can still recover,
// the wheels can no longer get back under the body. Driving them then does no good: they spin
// against the floor, or drag and shove the robot that is already lying on it. Zero output is the
// least harmful thing the motors can do. So past the limit the motor output is zeroed and STAYS
// zeroed (latched) until a person deliberately re-arms it. It never re-arms by itself, not even
// when the robot is picked up and the pitch comes back inside the limit.
//
// This is the OPPOSITE of C-3's rule (DiffDriveController.py): zeroing the wheels is safe here only
// because the robot has already fallen. While it is upright, a lost gamepad link zeroes only the
// velocity REFERENCE, and the balance loop keeps driving the wheels to hold it up.
//
// The limit (tipover_pitch_deg, proposed 35 deg) comes from the H-4 write-up in
// balance-loop-findings.md, not from a measurement of this robot's physics. Replace it with the
// angle measured in a tethered test.
//
// The rules, all in update():
//   - Armed, and |pitch| > limit             -> disarm (trip). Counted in trips().
//   - Armed, pitch NaN                       -> nothing. NaN means "no good reading": the IMU has
//                                               not produced one yet, or it has faulted. That is
//                                               handled elsewhere (finding C-2: the IMU plugin
//                                               returns ERROR; finding H-7: a dead upstream stops
//                                               the wheels). Tripping on NaN would make every
//                                               start-up a trip, because pitch starts as NaN.
//   - Disarmed, no re-arm request            -> stay disarmed, whatever the pitch.
//   - Disarmed, re-arm requested, |pitch| <= rearm_max -> re-arm.
//   - Disarmed, re-arm requested, otherwise  -> stay disarmed (refused). A NaN pitch is refused too:
//                                               the robot must be SEEN near upright.
//   A re-arm request is used up by the next update() whatever happens, so a request sent while the
//   robot is armed cannot quietly re-arm a later trip.
//
// Hysteresis: rearm_max (rearm_max_pitch_deg, proposed 10 deg) is well inside the trip limit. A
// robot re-armed at 34 deg would start driving while it still cannot recover, and trip again a
// moment later after a lurch. Requiring |pitch| <= rearm_max means a person has stood it up first.
//
// limit_deg <= 0 (or NaN) DISABLES the cutoff: is_enabled() says false and update() never trips.
//
// Threads: request_rearm() may be called from any thread (a ROS subscription callback); it only
// stores an atomic flag. Everything else is called from the control loop only. No allocation, no
// locks, no logging, no exceptions: safe at 100 Hz.
class TipoverLatch
{
public:
  // What changed in one update() call, so the caller can log it once.
  enum class Event { kNone, kTripped, kRearmed, kRearmRefused };

  // False for limit_deg <= 0 and for NaN (any comparison with NaN is false).
  static bool is_enabled(double limit_deg) { return limit_deg > 0.0; }

  // True if rearm_max_deg can be used with this (enabled) limit: above 0 and below limit_deg. A
  // re-arm band at or past the trip limit would re-arm a robot that cannot recover. False for NaN.
  static bool is_valid_rearm_max(double rearm_max_deg, double limit_deg)
  {
    return rearm_max_deg > 0.0 && rearm_max_deg < limit_deg;
  }

  // Any thread. The next update() acts on it, then clears it.
  void request_rearm() { rearm_requested_.store(true); }

  // Control loop, once per cycle. pitch_deg: this cycle's pitch [deg], NaN if unknown.
  // limit_deg: the cutoff angle [deg], applied to |pitch| (forward and backward alike).
  // rearm_max_deg: a re-arm is accepted only with |pitch| <= this [deg]; the caller checks it with
  // is_valid_rearm_max() once, at configure time.
  Event update(double pitch_deg, double limit_deg, double rearm_max_deg)
  {
    // Use up the request every cycle, armed or not (see the class comment).
    const bool rearm_requested = rearm_requested_.exchange(false);
    if (!is_enabled(limit_deg)) {
      return Event::kNone;
    }
    if (!disarmed_) {
      // NaN > limit is false, so a NaN pitch never trips.
      if (std::fabs(pitch_deg) > limit_deg) {
        disarmed_ = true;
        ++trips_;
        return Event::kTripped;
      }
      return Event::kNone;
    }
    if (!rearm_requested) {
      return Event::kNone;
    }
    // NaN <= rearm_max is false, so a NaN pitch refuses the re-arm.
    if (std::fabs(pitch_deg) <= rearm_max_deg) {
      disarmed_ = false;
      return Event::kRearmed;
    }
    return Event::kRearmRefused;
  }

  // True unless the cutoff has tripped and not been re-armed.
  bool armed() const { return !disarmed_; }

  // Trips since construction; reset() does not clear it.
  unsigned long trips() const { return trips_; }

  // Back to armed, with no pending request. The controller calls it on activation: activating
  // the controller is itself a deliberate operator action, and if the robot is still past the
  // limit the first update() trips again before any non-zero command goes out.
  void reset()
  {
    disarmed_ = false;
    rearm_requested_.store(false);
  }

private:
  bool disarmed_ = false;                   // latched: true from a trip until a re-arm
  unsigned long trips_ = 0;                 // [trips] since construction
  std::atomic<bool> rearm_requested_{false};  // set by request_rearm(), used up by update()
  static_assert(
    std::atomic<bool>::is_always_lock_free,
    "rearm_requested_ is read in the real-time loop and must not take a lock");
};

}  // namespace rocko_env

#endif  // ROCKO_ENV_TIPOVER_LATCH
