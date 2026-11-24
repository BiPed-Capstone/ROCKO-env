# Row 4.10 / finding U-2: one encoder service per wheel, speed over the measured time

- **Date / commit:** 2026-11-23, built on `95a8b23` (uncommitted when written; the orchestrator commits)
- **Branch:** ai-forward
- **Environment:** laptop (host tests) and the dev container (Tiers 0 to 3). Nothing ran on a Pi.

## Goal and success criteria

Fix U-2: `QuadEncoders.py` served one service to both motor plugins, so it ran twice per cycle, gave both wheels the
left encoder's position, and divided by a fixed 0.01 s.
1. Two services, `left_encoder_data` and `right_encoder_data`; each returns its own wheel's position [m] and velocity [m/s].
2. Velocity = position change since that wheel's previous call / measured elapsed time, then the 0.05 filter; the first
   call and dt <= 0 are safe.
3. `Motor12VoltQuadEncoder` reads the service name from the `encoder_service` hardware parameter and refuses to start
   without it; both xacros set it; `verify_urdf.sh` catches a missing or swapped one.
4. The sim's `model_encoder_node` copies the new node.
5. Host tests for the node (`tools/quad_encoders_test.py`) and the sim copy; compile-only check of the motor plugin.

## Plan

`QuadEncoders.py`; `Motor12VoltQuadEncoder.cpp/.hpp` (a section 6 file, **not compiled in the container**: the
compile-only check); both ros2_control xacros; `verify_urdf.sh`; `SimPlantModel.hpp`, `SimPlant.cpp`; tests; docs.
Control-loop self-check: the change touches `on_init` only in the plugin (read-only parameter lookup and the
service name; `read()` and `write()` are unchanged, so the blocking call without timeout, finding C-1, is as it was
and no worse). The concept: the velocity loop compares a commanded speed with a measured one, so if the measured
speed reads half the truth the loop's effective gain is doubled without anyone choosing that. No measurement of the
Pi's timing was made. Not gated: no gain, limit, pin or safety behaviour changed.

## What changed

- `QuadEncoders.py`: services `left_encoder_data` / `right_encoder_data` (the old `encoder_data` is gone), a small
  `WheelChannel` class per wheel (position, last time from `time.monotonic()`, filter state). Removed the unused
  feedforward subscriptions and the commented-out code, and the `numpy` import (`math.pi`): the subscriptions only
  fed commented-out lines, nothing read the values. Why the old node halved the speed: both plugins called the
  one service each cycle; the second call, right after the first, measured about 0 m/s, so the shared filter's
  mean settled near half the true speed.
- `Motor12VoltQuadEncoder.cpp/.hpp`: `encoder_service` parameter, `on_init` fails with a FATAL log if it is missing.
- `rocko.ros2_control.xacro`, `rocko_tank_mode.ros2_control.xacro`: `left_encoder_data` on the left motor block,
  `right_encoder_data` on the right (matched by joint).
- `verify_urdf.sh`: `encoder_services` check in every real (mock=false) expansion.
- `SimPlantModel.hpp` / `SimPlant.cpp`: `update_encoder_outputs(period_s)`: travel over the physics time that `read()`
  advanced, then the 0.05 filter; `period_s <= 0` holds. One filter serves both wheels (they are identical in the sim).
- `sim/scenario_checks.py`, `verify_sim.sh`: removed `velocity_from_true_periods` (it undid the old fixed-0.01 s error
  for scenario V; the replica now divides by the real time, so it would refuse to judge); V's overshoot is the
  exported peak. Comments updated. Threshold values are untouched.
- Docs: `sim-plant.md` (U-2, section 3.7, tables), `sim-to-real-gap.md`, `verification-ladder.md`, `measure-inertia.md`
  (service name), proposal 0010 wording, `next-steps.md` row 4.10.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `python3 -I tools/quad_encoders_test.py` | 8/8 pass. Red on 3 deliberate breaks (fixed `dt = 0.01`: 2 tests fail; right reply reads the left encoder: 1 fails; `dt <= 0` guard off: ZeroDivisionError), then restored |
| host | `python3 -I tools/imu_node_test.py` | 7/7 pass |
| host | `tools/sim_plant_model_test.cpp` (check #13: true speed at 10 and 20 ms cycles, hold at period 0) | pass. Red with the old fixed `/ 0.01` (the 20 ms case read 2x), then restored |
| 0 static | `verify_urdf.sh` | pass. Red when the tank xacro's right motor named `left_encoder_data` (`RightDriveMotor ... expected 'right_encoder_data'`), then restored |
| compile | `check_motor_plugin_compiles.sh` (in the container, via the ladder) | pass. Syntax and types only |
| 1, 2, 3 | `tools/run_ladder.sh` | build, verify_urdf, verify_mock both, verify_supervisor, verify_sim, verify_tools, all host tests, hooks, drift check: PASS. **Scenarios: FAIL, see below.** Doc link scan: failed only because this file did not exist yet |

### Scenarios before and after (encoder_sign -1 assumed; the model, not the robot)

Before = ladder run of 2026-11-23 19:51 (`95a8b23`, halved velocity). After = this change. `repeat_check.sh` 3 runs each:
V failed 3 of 3 (48.3, 48.7, 47.5 %), AB failed 3 of 3 (35 deg at 2.468, 2.470, 2.472 s): deterministic, not flakes.

| Scenario | Before | After |
| --- | --- | --- |
| U | PASS; late `\|pitch\|` <= 0.058 deg, `\|v\|` <= 0.0015 m/s | PASS; <= 0.098 deg, <= 0.0031 m/s (the U thresholds are unchanged) |
| I | PASS; peak 6.353 deg, back under 2 deg after 0.759 s | PASS; peak 11.985 deg, 0.571 s |
| V | PASS; 90 % in 1.120 s, overshoot 14.4 % (exported 20.8 %), peak pitch 2.996 deg | **FAIL**; 90 % in 0.499 s, overshoot 47.2 % in the ladder run and 47.5 to 48.7 % in the 3 repeats (limit 30 %), peak pitch 3.456 deg in the ladder run |
| K | FAIL; overshoot 66.4 % | PASS; 90 % in 1.488 s, overshoot 17.2 % |
| S2b | ok (35 deg at 0.141 s) | ok (0.142 s) |
| AB | ok (35 deg at 3.773 s, floor 3.0 s) | **FAIL**: 35 deg at 2.471 s in the ladder run (2.468 to 2.472 s in the 3 repeats), "sooner than 3.0 s" |

Reading: the loops were tuned against a velocity signal half the true size. With the true speed the outer loop's gain
doubles, so the step response is faster and overshoots more (V), the push recovery swings further (I), and the
H-12 run-away happens sooner (AB). K now passes (overshoot 17 %); the cause of its one earlier red was not investigated. This is a **finding, not a regression to patch**: the gains in `rocko_sim_controllers.yaml` are
gated and were not changed, and no threshold was moved. A human or the orchestrator decides whether to retune
the sim gains for the true speed (the outer p 25 is the first suspect) or to loosen AB's floor; the hardware gains
were tuned elsewhere and are untouched.

## Review rounds

Not run (the task was to implement and verify; the orchestrator runs the reviewers).

## Not verified, needs a human

- Nothing about the robot. The Python node was exercised only with stand-ins (no real `Encoder` module, no DDS).
- `Motor12VoltQuadEncoder.cpp` is **not built in the container**; only the compile-only check ran. It needs a Pi build.
- Procedure on the Pi (tethered, wheels off the ground), after a build there: start tank mode, check
  `ros2 service list` shows `/left_encoder_data` and `/right_encoder_data` and not `/encoder_data`; spin one wheel by hand
  and confirm only that wheel's `position` in `/joint_states` changes (this is the old bug's symptom: both used to follow
  the left); drive both wheels at a steady command and compare the reported speed with a measured wheel speed (a mark and
  a stopwatch). Abort if the plugin logs "does not have a 'encoder_service' parameter" or hangs waiting for a service
  (a stale install of the old node is the likely cause).
- Balancing mode must not be run on the robot on these gains before the sim finding above is settled (row 2.7, proposal 0006).
