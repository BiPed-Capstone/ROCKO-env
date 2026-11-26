# Proposal 0004 — confirm the proposed safety thresholds (watchdog, tipover, IMU faults, motor)

- **Status:** Open
- **Updated 2026-11-26:** file names are current (the IMU plugin was `ICM20948` until 2026-11-26; the range limits
  moved to `ImuSnapshotCheck.hpp`). Two facts in the table are older than the code: C-1 is fixed (the IMU is read
  inside the plugin over I²C), and the laptop heartbeat `HEARTBEAT_PERIOD_S` went with `Joystick.py`; the commands
  now come from `BalanceTeleop.py`/`TankTeleop.py` at the joy node's republish rate (`gamepad.py`).
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
| `command_timeout_s` | `DiffDriveController.py` (`DEFAULT_COMMAND_TIMEOUT_S`) | 0.2 s | no new gamepad command for this long => zero the velocity reference (C-3). Was set against the laptop heartbeat (`HEARTBEAT_PERIOD_S`, 0.05 s), now gone: see the 2026-11-26 note |
| `tipover_pitch_deg` | `add_feedforward_controller_parameters.yaml` | 35 deg | \|pitch\| past this zeroes the wheel command and latches (H-4); from finding H-4, not measured; includes the unexplained +7.6 deg offset |
| `rearm_max_pitch_deg` | same | 10 deg | a re-arm is accepted only inside this (hysteresis) |
| `kMaxConsecutiveImuFailures` / `kMaxWindowImuFailures` (the latter is `ImuFaultTracker::kMaxWindowFailures`) of `kWindowReads` | `BNO055.hpp` (the two limits); `ImuFaultTracker.hpp` (`kWindowReads`, `kMaxWindowFailures`) | 5 in a row / 10 of 20 reads | IMU read ERROR (C-2); 5 cycles is nominally 50 ms at 100 Hz |
| `kMaxAbsPitchRollDeg` / `kMaxAbsYawDeg` / `kMaxAbsPitchRateDegS` | `ImuSnapshotCheck.hpp` | 180 / 360 / 2000 | readings beyond these count as failed reads |
| `kMaxFeedforwardAgeNs` | `AddFeedforwardController.hpp` | 200 ms (200'000'000 ns) | the feedforward term becomes 0.0 once its last message is older than this (finding H-11); the reference still goes through. Matches `command_timeout_s`; `FF_AGE_S` in `verify_sim.sh` must equal it (`sim/check_sim_config.py` checks) |
| `kMaxStaleReferenceCycles` | `AddFeedforwardController.hpp` | 3 cycles | a missing chained reference holds the last command this long, then writes 0.0 |
| `kMaxConsecutiveWriteFailures` | same | 5 cycles | failed `set_value()` cycles before ERROR (M-5) |
| `kDeadbandRadPerSec` / `kStictionDutyPercent` | `Motor12VoltQuadEncoder.hpp` | 0.05 rad/s / 2 % | below the deadband the motor gets duty 0; above it the old +2 % offset is kept as an explicit stiction term (H-2) |
| `pitch_rate_axis` / `pitch_rate_sign` | hardware parameters of the Gyro block in `rocko.ros2_control.xacro` (since 2026-11-24; the gyro is read every cycle) | 0 / -1 | **measured** on the robot 2026-11-23 ([journal](../journal/2026-11-23-gyro-check.md)); not a threshold to confirm |

**Checked 2026-11-04 against `77664f4`:** every safety constant in `AddFeedforwardController.hpp`,
`ICM20948.hpp`, `ImuFaultTracker.hpp` and `Motor12VoltQuadEncoder.hpp` is in the table above. Left out
on purpose, because they only throttle log lines and decide no behavior: `kRefusalLogMinCycles` and
`kTimeoutLogMinCycles` (100 cycles each) and `kLogThrottleMs` (1000 ms). `kMaxFeedforwardAgeNs` was
already listed. The sim-only values (`fallen_pitch_deg`, the scenario thresholds in `verify_sim.sh`)
are not robot thresholds and are not proposed here. The candidate gains of
[proposal 0008](0008-candidate-gains-and-tethered-test.md) depend on these thresholds being confirmed
first.

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
