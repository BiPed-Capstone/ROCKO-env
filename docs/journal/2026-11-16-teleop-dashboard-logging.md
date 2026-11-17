# Readiness review, ramped drive commands, keyboard driving, a web dashboard and logging

- **Date / commit:** started 2026-11-16, built on `3d8dd97`. Each stage below names the commit it was
  built on.
- **Branch:** `ai-forward`.
- **Environment:** the `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop on macOS) and the
  laptop (Python 3, standard library). **Nothing ran on a robot or a Raspberry Pi.**

> ## HANDOFF: where this run stands (a new session starts here)
>
> **Updated after every stage, in the commit that finishes it.** To pick the run up: read this box,
> then the section of the first stage that is not `done`, then follow `CLAUDE.md` §2 for that stage.
> The plan the person approved on 2026-11-16 is in [Plan](#plan); do not re-ask what it settles.
>
> | Stage | What | Status |
> | --- | --- | --- |
> | 0 | Readiness review and the full ladder | done (this file, [Stage 0](#stage-0--readiness-review-2026-11-16)) |
> | A | Ramp the drive commands in `DiffDriveController.py`; sim scenario `K` | done, awaiting robot ([Stage A](#stage-a--ramp-the-drive-commands)); [proposal 0012](../proposals/0012-command-ramp-and-watchdog-step.md) open |
> | B | Keyboard driving from the laptop (`joystick/keyboard_teleop.py`) | not started |
> | C | Bag recording on the Pi (`record:=true`), metadata, `tools/fetch_logs.sh` | not started |
> | D | Web dashboard (replaces the Flask stub), with Record/Stop | not started |
> | E | Foxglove layout and guides | not started |
>
> **Next action:** start stage B (keyboard driving). Read `joystick/computer_joystick.py`,
> `Joystick.py` and `tools/computer_joystick_heartbeat_test.py` first. Note for B: the keyboard's
> commands now pass through stage A's ramp, and `Joystick.py`'s deadband and `linear_scale` make the
> smallest gamepad command 0.5 m/s, which is why B adds a "precise mode".
>
> **Uncommitted work when this was written:** `joystick/keyboard_teleop.py`, a first draft for stage
> B, untested and not wired to anything (`Joystick.py` has no precise mode yet). Stage A is committed
> (the commit after `8af19dd`) and pushed.
>
> **Decisions the person made (2026-11-16), so nobody asks again:** keyboard = a laptop script on the
> existing TCP protocol (not the browser, not ROS on the laptop); web view = a custom dashboard on the
> Pi **and** a Foxglove layout; logging = a bag on the Pi behind a launch argument **and** Record/Stop
> in the dashboard; bug scope = fix only what Tiers 0 to 3 can check, gated items stay proposals; the
> stage order A to E; "after each milestone the agent records its progress" (this box).
>
> **Decisions left to the agent's recommendation (the person did not object):** the keyboard sets a
> spare button byte as a "precise mode" flag so `Joystick.py` uses a low keyboard scale; the dashboard
> defaults to off on the robot launches and on in the sim; the watchdog's step to zero on comms loss
> is left as it is, with a proposal about ramping it.

## Goal and success criteria

The person asked (2026-11-16) for a review of the plan and the robot's state, a run in the simulation,
a verdict on readiness for real-world testing, fixes for bugs that would affect balancing, and then
three features: keyboard driving from another machine, robot data in a web interface on another
machine, and data logging and capture.

Checkable criteria, one block per stage:

- **Stage 0.** The whole ladder is run from a clean build and its results written here; the verdict
  names what blocks free-standing tests.
- **Stage A.** A new scenario `K` (a 0.5 m/s step, the smallest non-zero command the gamepad path can
  produce) is red on the code before the change and green after it; `U`, `I`, `V`, `S2b`, `AB` and the
  default `verify_sim.sh` still behave as before; the watchdog check `C3` still passes unchanged.
- **Stage B.** A host test drives the keyboard script's sender over a socket pair; a sim launch with
  `teleop:=true` receives its packets over TCP in the real `Joystick.py` and the velocity reference
  follows; the gamepad's behavior is unchanged (existing host tests pass).
- **Stage C.** A sim launch with `record:=true` produces an MCAP bag in which every listed topic has
  messages, plus a metadata file; the check goes red when a listed topic is missing.
- **Stage D.** With the sim running, the dashboard's data endpoint returns live pitch and wheel values
  that match `/dynamic_joint_states`, the page loads with no network access beyond the robot, and
  Record/Stop produce a bag that `ros2 bag info` can read.
- **Stage E.** Every topic the Foxglove layout names exists in the sim launch; the guide's steps are
  complete. How the layout looks in Foxglove is for a person.

## Plan

1. **A, ramp the drive commands.** Slew-limit the linear and angular command in
   `DiffDriveController.py` with named parameters, replacing the limiter code whose output is never
   used. Pick the limit from a sim sweep; the value is **proposed**. Scenario `K` in
   `verify_sim.sh scenarios`. The watchdog's step to zero stays; ramping it is a proposal (it changes a
   safety behavior, `CLAUDE.md` §3). Reviewers: `ros2-control-reviewer`, `explainability-reviewer`,
   `docs-reviewer`.
2. **B, keyboard.** `joystick/keyboard_teleop.py`, standard library, the gamepad's 40-byte packet and
   0.05 s heartbeat. Keys change the speed in small steps (a terminal sees no key releases); space
   stops; quitting sends zero. `Joystick.py` gains a "precise mode" read from a spare button byte, with
   a low keyboard scale as parameters; gamepad packets are handled as before.
3. **C, logging.** `record:=true` (plus `label`, `battery_v`, `surface`) on the launch files runs
   `ros2 bag record` in MCAP on a fixed topic list into `~/rocko_logs/`; a metadata file per run
   ([reproducing-results.md](../guides/reproducing-results.md)); a free-disk guard;
   `tools/fetch_logs.sh`.
4. **D, dashboard.** A standard-library HTTP server node replacing `webserver/app.py` (no Flask, no
   CDN): live values, status, plots at about 20 Hz, Record/Stop and downloads. It cannot drive the
   robot. Launch argument `dashboard:=`, default off on the robot, on in the sim.
5. **E, Foxglove.** A committed layout, a guide, and `foxglove:=true` in the sim launch.

Risks: the Pi's CPU load from the dashboard and the recorder is unmeasured and cannot be measured
here; the sim is planar, so nothing about turning can be checked; every sim result is about a model
with estimated parameters.

## Stage 0 — readiness review (2026-11-16)

Built on `3d8dd97`, clean tree. Docker was not running at the start; it was started and
`./check_container_mount.sh` printed `ok`.

### Verification

`tools/run_ladder.sh --clean` (one run): `RESULT: PASS`.

| Tier | Command | Result |
| --- | --- | --- |
| 1 build | `build_ros.sh` after `rm -rf build install log` | pass, 0 compiler warnings (21 s). The motor plugin is not compiled in the container |
| 0 static | `verify_urdf.sh`; hook tests; `sim/check_sim_config.py`; `tools/check_doc_links.py` | pass |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | pass |
| not a tier | `check_motor_plugin_compiles.sh` | pass (syntax and types only) |
| 3 sim | `verify_sim.sh` | pass: `S1 S2a H4 C3 C2 H6 FF` (132 s) |
| 3 sim | `verify_sim.sh scenarios` | pass: `U I V`, with `S2b` and `AB` ok (80 s) |
| host tests | the five C++ and three Python tests in `tools/` | pass |
| 4 robot | | **not run** (a human only) |

### Verdict: not ready for free-standing tests

Nothing new blocks it; all of this is in [next-steps](../next-steps.md) and needs a person or the
robot: C-1 (blocking sensor calls in the 100 Hz loop, proposal 0010), the encoder sign (0006), the
gains (0008), the measurements (0007), the safety thresholds (0004). The robot is ready for the stand
and tethered sequence in proposals 0006 and 0008, and nothing beyond it.

### New finding: a drive command is a step, and a step topples the model (H-13)

`DiffDriveController.command_controller()` computes a rate limit (`smooth_left_vel`,
`smooth_right_vel`) from `self.left_vel` and `self.right_vel`, which are always 0, and never uses the
result. A command therefore reaches the velocity loop's reference and the feedforward as a step.
Scenario `V` only tests 0.1 m/s. The smallest non-zero command the gamepad path can produce is
0.5 m/s: `Joystick.py` zeroes anything under a 0.05 deadband and multiplies by `linear_scale = -10`.

Measured in the sim (one run each, a scratch copy of `verify_sim.sh` with only `V_STEP_M_S` changed,
`verify_sim.sh scenarios V`, sim gains, `encoder_sign:=-1` assumed, estimated parameters):

| Step | Result in the model |
| --- | --- |
| 0.1 m/s (the committed `V`) | pass |
| 0.5 m/s | stays up; peak lean 21.1 deg (limit 10 deg); overshoot 20.8 % |
| 1.0 m/s | falls: 35 deg 0.168 s after the step |

This is about the model. Whether the robot does the same is not known. Turning could not be checked:
`SimPlant` is planar.

## Stage A — ramp the drive commands

Built on `8af19dd` (the commit that added this journal and scenario `K`, red, with its thresholds
fixed before the ramp existed).

### What changed

| File | What and why |
| --- | --- |
| `controllers/diffdrive/DiffDriveController.py` | The forward command moves toward the operator's by at most `max_linear_accel_m_s2` x 10 ms per cycle (`step_toward`). The two `RateLimiter` objects, whose output was never used, are gone. A time-out is still a step to zero and the ramp restarts from zero. The turn command is not ramped |
| `bringup/launch/rocko_sim.launch.py` | Argument `max_linear_accel_m_s2`, passed to the node, so `K` can be run with the ramp off |
| `verify_sim.sh` | Scenario `K`: V with a 0.5 m/s step and 5 s to reach 90 %; `K_WATCHDOG_S`; the optional `K_MAX_LINEAR_ACCEL_M_S2` |
| `sim/scenario_checks.py` | `--watchdog-trip-cannot-judge` for `velocity-step` (K uses it; V does not) |
| `sim/check_sim_config.py` | Check 7: the sim launch's ramp default equals the node's |
| `tools/teleop_watchdog_test.py` | Six ramp tests; the watchdog test runs with the ramp off; the `RateLimiter` stand-in removed |
| `tools/repeat_check.sh`, `tools/run_ladder.sh` | `repeat_check.sh "scenarios K" N` works (the check name is passed as words; its header's `V 3` example never worked); the ladder's label names K |
| docs | Finding H-13; proposal 0012; next-steps rows 1.16 and 4.1 to 4.4; the ladder; `control-cascade.md`; README |

`RateLimiter.py` is left in the tree; no node imports it now.

**Why a plain ramp and not the existing `RateLimiter`:** one line a student can read
(`step_toward`), against a 140-line port whose jerk limit nobody had tuned.

### Choosing the value (Tier 3, the model)

Scenario `K`, one run per line unless said; runs in which the watchdog fired are left out here and
listed below. Limits: lean 10 deg, overshoot 30 %, 90 % within 5 s.

| `max_linear_accel_m_s2` | Peak lean [deg] | Overshoot [%] | 90 % after [s] | Verdict |
| --- | --- | --- | --- | --- |
| 1000 (ramp off), 3 runs | 21.1 | 19.9, 20.1 (third not noted) | 0.83, 0.84 | FAIL |
| 2.0 | 17.0 | 54.7 | 0.99 | FAIL |
| 1.0 | 11.8 | 54.2 | 1.13 | FAIL |
| 0.7 | 10.0 | 22.5 | 1.29 | pass, on the limit |
| 0.5, 3 runs | 8.1 to 8.2 | 21.8 to 25.7 | 1.42 to 1.45 | pass |
| **0.35**, 9 runs (1 while choosing, 8 with `repeat_check.sh`) | 6.3 to 6.4, once 8.0 | 19.3 to 23.0 | 1.69 to 1.73 | pass 9 of 9 |
| 0.25 | 4.8 | 19.8 | 2.16 | pass |

**0.35 m/s^2** was chosen: about a third under the lean limit, in the middle of the passing range.
It is a **proposed** value ([proposal 0012](../proposals/0012-command-ramp-and-watchdog-step.md)).
The one 8.0 deg run among the eight is not explained.

**A second finding on the way.** In 3 of the first 8 runs at 0.2 s watchdog timeout the harness
stalled, the watchdog fired mid-drive, and the command stepped to zero and ramped again. Those runs
leaned 16.3 deg (0.5), 19.3 and 20.4 deg (0.25). So the watchdog's own step to zero disturbs the model
about as much as the step H-13 is about. It was not changed (a safety behavior): proposal 0012,
decision 2. `K` is launched with a 1 s timeout so that it measures the ramp, and reports *could not
judge* if the watchdog fires anyway.

### Verification

| Tier | Command | Result |
| --- | --- | --- |
| 1 build | `build_ros.sh` (via `tools/run_ladder.sh`) | pass, 0 compiler warnings |
| 0 static | `verify_urdf.sh`; hook tests (81 cases); `sim/check_sim_config.py` (7 `ok`); doc links | pass |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | pass |
| not a tier | `check_motor_plugin_compiles.sh` | pass |
| 3 sim | `verify_sim.sh` | pass: `S1 S2a H4 C3 C2 H6 FF` (151 s); C3 is unchanged and still requires exactly zero after a time-out |
| 3 sim | `verify_sim.sh scenarios` | first run: `K` could not judge (watchdog fired, 0.2 s timeout), the rest pass. After `K_WATCHDOG_S`: PASS, `U I V K` pass, `S2b` and `AB` ok |
| 3 sim | `tools/repeat_check.sh "scenarios K" 8` | pass 8 of 8 |
| host | `tools/teleop_watchdog_test.py` | 29 of 29 (28 before the review fix) |
| 4 robot | | **not run** |

Seen to fail: `K` with `K_MAX_LINEAR_ACCEL_M_S2=1000` (21.1 deg); the host tests with `step_toward`
returning its target (4 fail) and with the time-out not stepping to zero (2 fail);
`check_sim_config.py` with the launch default changed to 0.5 (`FAIL  the drive command ramp differs`).
A mistake on the way: the first ladder and repeat runs printed failures because their log folders did
not exist; nothing had run, and they were run again.

### Not verified — needs a human

- Anything on the robot. The model has estimated parameters and an assumed encoder sign.
- Turning: not ramped, and no tier can check it.
- What the ramp does to the gamepad's feel: full stick (10 "m/s") now takes about 29 s to reach, which
  is a limit on how fast the command changes, not a decision that 10 m/s is sensible (U-5).
- `--watchdog-trip-cannot-judge` was seen to trigger (the first full scenarios run) but V does not use
  it, so V can still fail on a watchdog lurch.
- The procedure for a person is in proposal 0012.

### Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | 3 Medium, 2 Low. (M) a NaN drive command made the new ramp run backwards without limit, because `min`/`max` do not propagate NaN; (M) `K_MAX_LINEAR_ACCEL_M_S2` was neither checked nor announced; (M) the watchdog-trip rule is tested before the fall, and counts a trip before the step; (L) the ramp is per cycle, so an upper bound; (L) K did not print its watchdog timeout | Fixed: a command that is not finite is dropped and does not feed the watchdog (host test added, red when the guard is removed; 29 of 29); the variable must be a number and K prints `RAMP CHANGED` and its watchdog timeout; the comment says upper bound. **Declined:** reordering the watchdog-trip rule. The trip times are not recorded with the flags, K's 1 s timeout makes a trip rare, and the ramp-off run was seen to exit 1 |
| 1 | `explainability-reviewer` | 2 Medium, 5 Low: two copies of a stall rate; the env override missing from the tunables block; comments (a literal 0.01, the fallback cases, an unchecked claim about turning stated as fact); a section header in the test file | Fixed, except `CYCLE_S` in the test file (declined: the file's existing `cycles()` helper already uses 0.01) |
| 1 | `docs-reviewer` | 2 High, 2 Medium, 4 Low: a blank line broke the next-steps table at row 1.16; `verify-in-container.md` and a `verify_sim.sh` comment still listed five scenarios and 22 tests; the HANDOFF box said nothing was uncommitted; proposal 0012 contradicted itself on where the value lives and told a person to `ros2 run` a second `DiffDriveController.py`; K missing from a sentence; proposal order; the sweep table copied into the proposal | Fixed (the guide now links to the ladder for the counts; the proposal links to this journal for the table and gives the machine, the shell and an edit-and-rebuild step). **Declined:** deleting `RateLimiter.py` (next-steps rows 0.4 and 0.9 are about it; removing a file is a separate decision) |
| 2 | not run | | The round 1 fixes were re-verified (`verify_sim.sh scenarios` PASS, `verify_sim.sh C3` PASS, `verify_urdf.sh` PASS, host test 29 of 29) but **not re-reviewed** by the agents |

### Reproduce

```bash
docker compose -f docker/compose.yml up -d
./check_container_mount.sh
tools/run_ladder.sh
tools/repeat_check.sh "scenarios K" 8
K_MAX_LINEAR_ACCEL_M_S2=1000 tools/repeat_check.sh "scenarios K" 1    # must fail
```

## Stage B — keyboard driving

Not started.

## Stage C — logging

Not started.

## Stage D — web dashboard

Not started.

## Stage E — Foxglove

Not started.
