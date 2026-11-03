# Proposal 0004 — confirm the proposed safety thresholds (watchdog, tipover, IMU faults, motor)

- **Status:** Open
- **Gate:** hardware gains and limits / safety behavior (values only a human can confirm on the robot)
- **Raised:** 2026-11-02, built on `85bdc31`, run journal:
  [2026-10-31-sim-plant-and-critical-fixes.md](../journal/2026-10-31-sim-plant-and-critical-fixes.md)

## The change

Nothing is edited in `bringup/config/rocko_controllers.yaml`. Every value below is a named constant or
a parameter default in code, added by the stage 2 to 4 work, and each is a **proposed value, not a
measured one**. Please confirm, change, or decline each one (a changed value is a one-line edit at
the place named).

| Value | Where | Proposed | Meaning |
| --- | --- | --- | --- |
| `command_timeout_s` | `DiffDriveController.py` (`DEFAULT_COMMAND_TIMEOUT_S`) | 0.2 s | no new gamepad command for this long => zero the velocity reference (C-3). Must be at least 4 x the laptop heartbeat (`HEARTBEAT_PERIOD_S`, 0.05 s) |
| `tipover_pitch_deg` | `add_feedforward_controller_parameters.yaml` | 35 deg | \|pitch\| past this zeroes the wheel command and latches (H-4); from finding H-4, not measured; includes the unexplained +7.6 deg offset |
| `rearm_max_pitch_deg` | same | 10 deg | a re-arm is accepted only inside this (hysteresis) |
| `kMaxConsecutiveImuFailures` / `kMaxWindowImuFailures` (the latter is `ImuFaultTracker::kMaxWindowFailures`) of `kWindowReads` | `ICM20948.hpp` (the two limits); `ImuFaultTracker.hpp` (`kWindowReads`, `kMaxWindowFailures`) | 5 in a row / 10 of 20 reads | IMU read ERROR (C-2); 5 cycles is nominally 50 ms at 100 Hz, longer while C-1 is unfixed |
| `kMaxAbsPitchRollDeg` / `kMaxAbsYawDeg` / `kMaxAbsPitchRateDegS` | `ICM20948.hpp` | 180 / 360 / 2000 | readings beyond these count as failed reads |
| `kMaxFeedforwardAgeNs` | `AddFeedforwardController.hpp` | 200 ms (200'000'000 ns) | the feedforward term becomes 0.0 once its last message is older than this (finding H-11); the reference still goes through. Matches `command_timeout_s`; `FF_AGE_S` in `verify_sim.sh` must equal it (`sim/check_sim_config.py` checks) |
| `kMaxStaleReferenceCycles` | `AddFeedforwardController.hpp` | 3 cycles | a missing chained reference holds the last command this long, then writes 0.0 |
| `kMaxConsecutiveWriteFailures` | same | 5 cycles | failed `set_value()` cycles before ERROR (M-5) |
| `kDeadbandRadPerSec` / `kStictionDutyPercent` | `Motor12VoltQuadEncoder.hpp` | 0.05 rad/s / 2 % | below the deadband the motor gets duty 0; above it the old +2 % offset is kept as an explicit stiction term (H-2) |
| `read_pitch_rate` | `ICM20948.py` node parameter | false | whether the node reads the gyro at all (finding M-7); false because no real controller uses the rate. Turn it on together with the gated YAML change (H-6): both are needed |
| `pitch_rate_axis` / `pitch_rate_sign` | `ICM20948.py` node parameters | 0 / +1 | **guesses**: which gyro axis is robot pitch and its sign (H-6); establish on the robot |

## Why

These values decide when a robot stops, when it gives up on its sensor and how fast it reacts to a lost
gamepad. A simulator with invented parameters cannot choose them.

## Evidence

Tier 1 (builds) and Tier 3 (model with estimated, unmeasured parameters): `verify_sim.sh` checks H4, C3,
C2, H6 and FF against these defaults. Host unit tests cover the logic of each. Not verified: any of the values
on a robot.

## Risk

Too small a tipover limit disarms a robot that could recover; too large a one thrashes on the floor. Too
short a watchdog timeout stops a robot whose Wi-Fi hiccups; too long leaves a stale command driving it.

## Test for a human

Tethered, wheels off the ground first. For each row, change the value, run the matching check in
[the journal's robot procedures](../journal/2026-10-31-sim-plant-and-critical-fixes.md), and record
commit SHA, launch file, battery voltage, surface and the log path
([reproducing results](../guides/reproducing-results.md)). Abort by cutting motor power at the first
unexpected wheel motion.

## Decision

(Human fills in.)
