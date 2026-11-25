# Make level read 0 (pitch offset), and propose a clamp on the target lean after the first floor test

- **Date / commit:** 2026-11-25, built on `2772ac5` (this entry is added by the commit that makes the change)
- **Branch:** `ai-forward`
- **Environment:** laptop + Docker dev container; the robot read once over `ssh rocko` with the owner present (then the
  owner switched it off)

## Goal and success criteria

After the owner's first balance test on a flat floor ("no matter where I leaned, the robot was spinning its wheels in
one direction; the trip detection worked in both directions"), the owner asked for two things: (1) the proposal for the
fix the agent suggested, and (2) to record the pitch offset: with the robot flat and level the pitch read 10.1.

1. The robot standing flat and level reports pitch 0.0: `pitch_offset_deg` in `rocko.ros2_control.xacro` corrected
   from a live reading, and every place that quotes the old 7.6 updated.
2. [Proposal 0017](../proposals/0017-clamp-the-target-lean.md): clamp the velocity loop's output (the target lean) and
   its integral share to ±5 deg, with the reasoning, the evidence and its limits, and a test.
3. The ladder stays green.

## What the floor test shows (from the service log; no recording)

Balance enabled twice from the dashboard (both passed the new hold-still rule); each time the tipover cutoff tripped at
35 deg, after about 22 s and 7 s. No pitch or wheel data was recorded, so the cause is reasoned, not measured: the
suspected cause is the velocity loop's integral term (`i = -5`, `i_clamp` ±150 deg of target lean) winding the target
lean up while the wheels spin, and `10 x target` then outweighs any real lean (proposal 0017, Why). The wrong pitch zero (level read +10.1 deg)
would have made the first command the same sign every time (reasoning, not seen).

## The pitch offset

Read on the robot (balance mode, disabled, the owner holding it flat and level), 3 s of `/dynamic_joint_states`:
291 samples, pitch 10.10 deg every one, pitch rate -0.06 deg/s. That pitch already includes the old offset of +7.6, so
the sensor reads +2.5 deg level (inferred: 10.10 minus the 7.6 the plugin added) and the new offset is **-2.5**. "Level" is the owner's flat-and-level position of the
body; the balance point (centre of mass over the axle) can differ and is not measured. Commit on the Pi: `2772ac5`;
battery voltage and surface not recorded.

## What changed

- `rocko.ros2_control.xacro`: `pitch_offset_deg` 7.6 → -2.5, with the measurement in the comment.
- `ICM20948.hpp`/`.cpp` (realtime review): the plugin's fallback offset was 7.6, used if the xacro left the parameter
  out; it is now NaN, so the existing finite check refuses to start without the xacro's value (one home for the
  number). `on_init` only; nothing in `read()` changed.
- Comments that stated 7.6 as the current value (`Bno055UartProtocol.hpp`, `SimPlantModel.hpp`, `tools/imu_tilt_log.py`, and): `AddFeedforwardController.cpp` (comment only), `add_feedforward_controller_parameters.yaml`
  (comment only), `ImuSnapshotCheck.hpp`, `tools/imu_snapshot_check_test.cpp` (its cases keep 7.6 as a test value, now
  labelled as the old value).
- Docs: imu-i2c-read's numbers table, sim-plant (pitch zero, the IMU paragraph, the bias row), sim-to-real-gap 3.12
  and its table, the ladder's tipover note, next-steps. Historical journal entries keep 7.6.
- `docs/proposals/0017-clamp-the-target-lean.md` (new) and its index row.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `imu_snapshot_check_test.cpp` (in `tools/run_ladder.sh`) | passes (unchanged behaviour) |
| 0, 1 | `build_ros.sh` and `verify_urdf.sh` on the final code | both pass. A full `tools/run_ladder.sh` was started before the `ICM20948` default change and had not reported when the session's usage limit ran out: **Tiers 2 and 3 not confirmed on this commit** |
| all | that full `tools/run_ladder.sh` (on the code before the `ICM20948` default change) | finished after the commit: **FAIL** in Tier 3, check `S2a-no-torque` only (`|pitch| reached 35.0 deg 0.622 s after it started to move (limit 0.48 s)`), everything else PASS. Not investigated (usage limit). Nothing in this change reaches the simulation (the offset is in the real-hardware block only), so it may be a timing flake: re-run with `tools/repeat_check.sh S2a 10` and keep it red until a cause is found |
| 4 | the new offset on the robot | **not run**: the owner switched the robot off before it could be pulled; on the next start, `git pull && ./build_ros.sh`; the `BNO055 ready: pitch ...` line `ICM20948.cpp` logs at configure should read about 0 with the robot level. **Before the next enable**, read the pitch at the robot's balance point; stop if it is near 10 deg (proposal 0017, test step 1) |

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | realtime-auditor | offset arithmetic correct, nothing assumed a positive offset; Medium: the balance point may now read far from 0; Low: the plugin's fallback offset was still 7.6; Low: stale 7.6 comments | balance-point check added before the next enable (proposal 0017 step 1, this journal); fallback now NaN (refuses to start without the xacro value); comments fixed |
| 1 | docs-reviewer | cause stated as fact in the index and next-steps, unsourced clamp semantics and bound, which gain the 10 is, the sim comment that goes stale, leftover 7.6 quotes, links | all fixed |
| 2 | realtime-auditor | clean (the NaN default, `on_init` only) | nothing to fix |

## Not verified — needs a human

The offset on the robot (next pull), the balance point, and proposal 0017 (the owner decides; its test is there).
