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
> | B | Keyboard driving from the laptop (`joystick/keyboard_teleop.py`) | done, awaiting robot ([Stage B](#stage-b--keyboard-driving)); guide: [drive-with-keyboard.md](../guides/drive-with-keyboard.md) |
> | C | Bag recording on the Pi (`record:=true`), metadata, `tools/fetch_logs.sh` | done, awaiting robot ([Stage C](#stage-c--logging)); guide: [record-a-run.md](../guides/record-a-run.md) |
> | D | Web dashboard (replaces the Flask stub), with Record/Stop | done, awaiting robot ([Stage D](#stage-d--web-dashboard)); guide: [use-the-dashboard.md](../guides/use-the-dashboard.md) |
> | E | Foxglove layout and guides | **partly done, NOT reviewed** ([Stage E](#stage-e--foxglove)): the code and the check are committed; the guide, the other docs and the review are not written |
>
> **Next action:** finish stage E. The run stopped here on 2026-11-17 because the session's usage
> limit was reached, not because anything failed. Left to do, in order: (1) show
> `sim/check_foxglove_layout.py` red on a deliberate break (misspell one field in
> `foxglove/rocko_balance_layout.json`) and restore it; (2) write `docs/guides/use-foxglove.md`
> (connect to `ws://<robot-ip>:8765`, import the layout, open a recorded `.mcap`, and a table of the
> layout's message paths so the panels can be rebuilt by hand, because **the layout's JSON structure
> was written from memory and has never been opened in Foxglove**); (3) update the ladder's
> `verify_tools.sh` section, `docs/guides/README.md`, the README and next-steps row 4.4; (4) run
> `tools/run_ladder.sh`; (5) review with `ros2-control-reviewer`, `explainability-reviewer` and
> `docs-reviewer`, fix, commit, push.
>
> **Uncommitted work when this was written:** none. Stages A to D and the stage E code below are
> committed and pushed. (If `git status` is not clean or `git log origin/ai-forward..HEAD` lists
> commits, that did not happen.)
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

Built on `f9fedbc`.

### What changed

| File | What and why |
| --- | --- |
| `joystick/keyboard_teleop.py` (new, laptop) | Standard library only. `KeyboardState` turns keys into a forward and a turn level (-5 to +5); `drive()` sends the gamepad's 40-byte packet on a change and every 0.05 s, and a stop on quit. It imports `pack` and the constants from `computer_joystick.py` instead of copying them |
| `controllers/joystick/Joystick.py` | Precise mode: when button byte 15 is 1, the axis is multiplied by `precise_linear_scale_m_s` / `precise_angular_scale_m_s` (ROS parameters; defaults are named constants, **proposed**) with no expo curve and no deadband. Gamepad packets take the old path, unchanged |
| `sim/scenario_checks.py`, `verify_sim.sh` | Scenario `KB`: the laptop script itself, with scripted keys, over TCP into the real `Joystick.py` |
| `tools/keyboard_teleop_test.py` (new), `tools/teleop_watchdog_test.py` | Host tests of the keys and the sending loop, and of `Joystick.py`'s two mappings |
| docs | The guide [drive-with-keyboard.md](../guides/drive-with-keyboard.md); README; the ladder; next-steps row 4.1; proposal 0012, decision 3 |

**Why a button byte and not a second port or a new packet:** it needs no protocol change, an old
`Joystick.py` simply ignores it, and the robot, not the laptop, decides how fast "full" is.
**Why keys set a speed:** a terminal reports no key releases.

### Verification

| Tier | Command | Result |
| --- | --- | --- |
| all | `tools/run_ladder.sh` | `RESULT: PASS` (build 0 warnings, urdf, both mocks, compile-only, `verify_sim.sh` 153 s, scenarios `U I V K KB` pass with `S2b`, `AB` ok, every host test, hook tests, drift check, links) |
| 3 sim | `tools/repeat_check.sh "scenarios KB" 5` | pass 5 of 5; peak lean 5.30 to 5.37 deg for three presses (0.3 m/s) |
| host | `tools/keyboard_teleop_test.py`; `tools/teleop_watchdog_test.py` | 5 of 5; 32 of 32 (4 and 31 before the review fixes) |
| by hand, laptop | the script in a pseudo-terminal against a local TCP listener, keys `w`, `wa`, `q` | 19 packets of 40 bytes, forward axis 0, -0.2, -0.4, button 15 always set, last packet a stop, exit 0, terminal restored |
| by hand, laptop, after the review fixes | the same, keys up-arrow, left-arrow, `ww`, Ctrl-C | 22 packets; forward axis 0, -0.2, -0.4 only; turn axis 0 only (the arrows did nothing); last packet a stop; exit 0 |
| 3 sim, after the review fixes | `verify_sim.sh scenarios` | PASS: `U I V K KB` pass, `S2b` and `AB` ok |
| 4 robot | | **not run** |

Seen to fail: `KB` with the precise button not set (`FAIL: ... speeds off the 0.1 m/s grid [1.14, 2.49,
4.27]`: the gamepad mapping) and with the precise scale doubled (`FAIL: 3 presses must command 0.3 m/s;
the peak was 0.6`); the host tests with the heartbeat removed, with quit not zeroing, and with each of
those two breaks.

A mistake on the way: the first pseudo-terminal test hung. The cause was the test, which did not read
the terminal's output, together with `tcsetattr(TCSADRAIN)` waiting for that output on exit. The script
now restores the terminal with `TCSANOW`, and reads keys with `os.read` instead of `sys.stdin.read`
(whose buffer could hold a second key back; plausible, not demonstrated).

### Not verified — needs a human

- Anything on the robot or over a real network, including whether the heartbeat keeps the 0.2 s
  watchdog quiet over Wi-Fi (`KB` runs with a 1 s timeout, for the reason K does).
- Turning: `a`/`d` reach `Joystick.py` (host test), but the sim is planar, so what a turn does to
  balance is unknown, and the turn scale has no evidence behind it.
- Windows. A real terminal beyond the one pseudo-terminal run.
- Whether "positive `linear.x`" is forward on the robot (the keyboard follows the gamepad's signs).
- Procedure: [drive-with-keyboard.md](../guides/drive-with-keyboard.md), first under the tethered
  conditions of proposal 0008; proposal 0012 asks a person to confirm the speeds.

### Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | 1 Medium, 3 Low. (M) an arrow key arrives as ESC `[` and a letter, and the letter was read as a key: up-arrow turned left and left-arrow turned **right**; (L) Ctrl-C raised `KeyboardInterrupt` and never sent the stop (the `\x03` branch was dead); (L) the precise scale parameters were not checked (a positive value reverses the keys); (L) `KB` did not check that the keyboard thread finished | All fixed: an ESC sequence is discarded (`read_key_from`, host test, red with the handling removed); `main()` sends a stop on Ctrl-C (seen in a pseudo-terminal: last packet zero, exit 0); a scale outside [-`MAX_PRECISE_SCALE_M_S`, 0) falls back to the default with an error (host test); `KB` exits 2 if the thread is still alive |
| 1 | `explainability-reviewer` | 3 Medium, 4 Low: `check_keyboard_drive` too long; a cryptic flags expression; axis and button indices written in two files; `KB_STEP_M_S` derived by hand; "angular" for a value in m/s; a repeated port number; injected `now` | Fixed: `connect_when_listening` extracted, `watchdog_fired` named, comments say KB fails if the indices or the step drift, the unit comment. **Declined:** promoting `Joystick.py`'s `linear_axis`/`angular_axis` locals to constants (it edits the gamepad path for no behavior; KB guards the drift); dropping `now` (the host test uses it) |
| 1 | `docs-reviewer` | 5 Low: a README scenario list; `--port` undocumented; the two laptop files must stay together; the 0.5 s replace rule; the HANDOFF box's "committed and pushed" | Fixed |
| 2 | not run | | The fixes were re-verified (host tests 5 of 5 and 32 of 32, `verify_sim.sh scenarios` and the pseudo-terminal run below) but **not re-reviewed** |

### Reproduce

```bash
tools/run_ladder.sh
tools/repeat_check.sh "scenarios KB" 5
python3 -I tools/keyboard_teleop_test.py
```

## Stage C — logging

Built on `9e8ecae`.

### What changed

| File | What and why |
| --- | --- |
| `rocko_env/recording/RecordRun.py` (new) | A plain program, not a node and not on the control path. Makes `<log_root>/<time>_<label>/`, writes `run.yaml` (commit and whether the tree is dirty, launch, `real`/`mock`/`sim`, the config's path and SHA-256, battery, surface; `not recorded` for anything left out), runs `ros2 bag record --storage mcap` on `RECORDED_TOPICS`, stops it with SIGINT so the bag is closed, and refuses to start or stops under 500 MB free |
| `rocko_env/recording/launch_recording.py` (new) | The `record`, `record_label`, `battery_v`, `surface`, `log_root` arguments and the recorder action, shared by the three launch files so they cannot drift |
| `rocko.launch.py`, `rocko_tank_mode.launch.py`, `rocko_sim.launch.py` | Use the above. `record` defaults to `false`: a plain launch is unchanged |
| `CMakeLists.txt`, `package.xml` | Install `RecordRun.py`; `ros2bag` and `rosbag2_storage_mcap` as run dependencies |
| `verify_tools.sh` (new), `sim/check_recording.py` (new) | The check: record in the sim, stop like Ctrl-C, judge the run folder |
| `tools/record_run_test.py` (new), `tools/fetch_logs.sh` (new), `tools/run_ladder.sh` | Host test; `rsync` the Pi's runs to a laptop; the ladder runs `verify_tools.sh` |
| docs | [record-a-run.md](../guides/record-a-run.md); `reproducing-results.md` now points at `record:=true`; the ladder; README; next-steps row 4.2 |

**Why one program and not `ros2 bag record` in each launch file:** the dashboard's Record button
(stage D) has to record the same topics and write the same `run.yaml`, so the list and the metadata
live in one place.

### Verification

| Tier | Command | Result |
| --- | --- | --- |
| 3 sim | `verify_tools.sh` | first run **FAIL**, a real fault: `bag/metadata.yaml is missing`. Started from a background script the recorder inherited "ignore SIGINT" and never closed the bag; `RecordRun.py` now resets it for the child. Then PASS: 5587 messages in 7.4 s, 5.17 MB (42 MB per minute, one measurement, in the container), `/dynamic_joint_states` 712 |
| host | `tools/record_run_test.py` | 3 of 3 |
| all | `tools/run_ladder.sh` | every step passed **except** `verify_sim.sh scenarios`: V failed (overshoot 46.0 %). See the next section |
| 3 sim, after the V fixes | `tools/repeat_check.sh "scenarios V" 10`; `tools/repeat_check.sh scenarios 2` | pass 10 of 10 (overshoot 12.4 to 17.4 %); pass 2 of 2 |
| all, after the review fixes | `tools/run_ladder.sh` | `RESULT: PASS`: build 0 warnings, urdf, both mocks, compile-only, `verify_sim.sh` (152 s), `verify_tools.sh`, scenarios `U I V K KB` pass with `S2b`, `AB` ok, every host test, hook tests, drift check, links |
| 4 robot | | **not run** |

Seen to fail: `verify_tools.sh` on its first run (above); with `/left_feedforward` commented out of
`RECORDED_TOPICS` (`FAIL  /left_feedforward has no messages in the bag`); and on the `ros2 run` wiring once
it signalled the launch alone (review round 1).

Two mistakes on the way, both mine. A check that signalled the launch process printed `FAIL: the launch
did not shut down` for the wrong reason: a `ros2 launch` started in the background by a script ignores
SIGINT altogether (a plain launch, no recording, was still running 15 s after the signal), so the check now
starts it with `env --default-signal=INT`. And two clean-up commands killed themselves, because
`pkill -f <pattern>` matched their own command line: the trap `tools/repeat_check.sh` was written to avoid.

### A red V, and its two causes

Stage C did not touch V's path, but the ladder's V failed, and a red result stays red until a cause is
found (`CLAUDE.md` §12). `tools/repeat_check.sh "scenarios V" 10` then failed 4 of 10.

1. **Three of the four (47.9, 52.4, 52.6 %) and the ladder's (46.0 %): the teleop watchdog fired
   mid-step.** Each of those logs has the note `command_timed_out went True while the gamepad was
   driving`; no passing run but one has it (25.9 %). The cause of the trip is the harness (the fake
   gamepad or the node stalls about 0.3 s in the container). The reason it now fails V is **stage A's
   ramp**: before it, a trip was a one-cycle dip (zero, then the step again); with it, the command
   restarts from zero and ramps for 0.29 s, a second transient. This is the same disturbance as
   proposal 0012's decision 2, seen from V. **Fix (the check, not the robot's code):** V is launched
   with the 1 s timeout K and KB already use (renamed `STEP_WATCHDOG_S`) and exits 2 if the watchdog
   fires anyway. Stage A should have done this for V when it did it for K.
2. **The fourth (34.9 %; then one at 70.7 % in the next ten): the overshoot metric, not the model.**
   The exported samples of those runs peaked at 19.1 and 17.2 %. `velocity_from_true_periods` takes the
   gap between two message stamps as the cycle's length. In both runs one message was stamped late: gaps
   of 17.1 then 2.6 ms, and 17.8 then 1.4 ms, each pair adding up to two normal cycles. A normal
   cycle's travel divided by 1.4 ms read as a sevenfold speed. **Fix:** a gap shorter than 10 ms is
   taken as one nominal period. Replayed on the kept samples: 70.7 % becomes 14.3 %, 34.9 % becomes
   14.0 %, and a genuine watchdog-lurch run stays at 51.0 % (was 52.4 %). The 30 % limit is unchanged.

After both: V 10 of 10, the scenarios mode 2 of 2. Not done: the sweep (`sim/parameter_sweep.py`) was not
re-run with the changed metric, so its tallies in the ladder are from the metric of 2026-11-07.

### Not verified — needs a human

- Anything on the Pi: that the recorder finds `ros2`, `git` and the checkout there, that
  `rosbag2_storage_mcap` is installed, and that `fetch_logs.sh` works (it was only syntax-checked:
  there is no robot to copy from).
- **What recording costs the 100 Hz loop and the SD card on the Pi.** Unknown. The procedure: run the
  same stand test twice, with and without `record:=true`, and compare the spread of cycle times
  (`ros2 topic echo /controller_manager/statistics/full`, or the recording itself for the run that has
  one). Until then a recorded balance run may not be representative.
- The low-disk stop while recording (only the refusal at start has a test).
- `tools/record_run_test.py` was shown to fail on one deliberate break only (the label not cut to 60 characters).

### Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | 1 High, 1 Medium, 2 Low. **(H) the recorder did not stop with the launch.** Launch started it as `ros2 run rocko_env RecordRun.py` and signals only that `ros2 run` process, which does not pass the signal on; except for Ctrl-C in a terminal (which signals everything), `RecordRun.py` and `ros2 bag record` kept running with the bag open until the disk guard. `verify_tools.sh` hid it by signalling the whole process group. (M) `fetch_logs.sh` used an `rsync` option macOS's rsync does not have. (L) `use_mock_hardware:=True` was labelled `real`. (L) launch's 5 s grace was shorter than the recorder's 15 s | All fixed. `verify_tools.sh` now signals the launch process alone and fails if anything it started is still running: **red on the old wiring** (`FAIL: processes outlived the launch`, listing `RecordRun.py` and `ros2 bag record`), green after `launch_recording.py` starts the recorder with `launch_ros`'s `Node` and a 20 s grace. `--progress` for rsync (the option is accepted on this Mac; no copy was made: there is no robot). `True` and `1` count as mock |
| 1 | `explainability-reviewer` | 4 Medium, 5 Low: the exit status came from searching the stop message for words; a dict used only to be mutated; the log folder default written three times; the docstring named a dashboard button that does not exist yet; dates a day ahead (the container's clock is UTC); `STEP_WATCHDOG_S` placed among K's constants; the `mock`/`real` expression copied into two launch files; a duplicated comment; no hint on editing the topic list | All fixed (`failed` flag; the default lives in `RecordRun.py`; `real_or_mock()` in the shared module; dates are local, 2026-11-16) |
| 1 | `docs-reviewer` | 3 Medium, 3 Low: a hand-started recorder leaves launch, hardware and config hash `not recorded` and the guide did not say so; `verify_tools.sh` missing from `verify-in-container.md` and the `build-verify` skill; the HANDOFF box said done while the review was pending; `rsync` needed on both machines; proposal 0008 still says `ros2 bag record -a`; dates | Fixed, except proposal 0008 (**declined**: a proposal is a record of what was proposed; switch it to `record:=true` when it is next revised) |
| 2 | not run | | The fixes were re-verified by `tools/run_ladder.sh` (result in the Verification table) but **not re-reviewed** |

### Reproduce

```bash
tools/run_ladder.sh          # includes verify_tools.sh
python3 -I tools/record_run_test.py
```

## Stage D — web dashboard

Built on `e873e26`.

### What changed

| File | What and why |
| --- | --- |
| `rocko_env/webserver/Dashboard.py` (new) | A ROS node plus Python's built-in web server in one process. Subscribes to four 100 Hz topics **raw**, keeps the newest bytes, and decodes one message per topic 20 times a second; keeps 60 s of history; serves the page, `/api/state`, `/api/history`, `/api/runs` and a run's `.tar`; `POST /api/record/start` and `/stop` run and stop `RecordRun.py`. No publisher, no service client. Lowers its own priority (`os.nice(10)`) |
| `rocko_env/webserver/dashboard.html` (new) | The page: tiles, three canvas plots, the record form, the run list, warnings. One file, no outside scripts or fonts |
| `rocko_env/webserver/launch_dashboard.py` (new), the three launch files | `dashboard:=` (default `false` on the robot's two launch files, `true` in the sim) and `dashboard_port:=` |
| `rocko_env/webserver/app.py`, `templates/` (deleted) | The Flask stub: it subscribed to two PWM topics nothing publishes, and its pages loaded a script from a CDN |
| `CMakeLists.txt` | Installs `Dashboard.py` and the page (`share/rocko_env/web/`) |
| `verify_tools.sh`, `sim/check_dashboard.py` (new) | The dashboard part of the check |
| docs | [use-the-dashboard.md](../guides/use-the-dashboard.md); README (the `flask` dependency row is gone); the ladder; next-steps row 4.3 |

**Why the standard library and polling:** the Pi's Python dependencies are unpinned (issue #14) and
the container has no Flask, so nothing new is installed; a page that asks for JSON a few times a second
is the simplest thing a student can read, against websockets or server-sent events.

### Verification

| Tier | Command | Result |
| --- | --- | --- |
| 3 sim | `verify_tools.sh` | first run **FAIL**: `only 37 history points overlap the 6.0 s`. Real: decoding every 100 Hz message in Python, the node fell behind and its 20 Hz history timer ran at about 6 Hz. Fixed by raw subscriptions (decode the newest message at 20 Hz): 19.6 points a second. Second run **FAIL**: `/dynamic_joint_states has 122 messages` (the HTTP recording ran 4 s; now 7 s). Then PASS: `110 history points within 0.011 deg of the robot's pitch (start-up lean 2.00 deg seen); publishes nothing; Record/Stop made ..._dashboard (3.74 MB); foreign Origin, a second Record and a climbing path refused` |
| 3 sim, by hand | the sim for 11 s, then `ps` | `Dashboard.py` 21 % of one core averaged since its start (start-up included); `DiffDriveController.py` 14 %, `Joystick.py` 10.5 %, `ros2_control_node` 19 %. One measurement, in the container, on a laptop: not the Pi |
| by hand, laptop | the real `dashboard.html` served by a throwaway mock with made-up data, in a browser | tiles, plots, the record form, the run list and the warnings render; log text containing `<b>` is shown as text; Record and Stop enable each other; no console errors |
| all | `tools/run_ladder.sh` (before the review fixes) | `RESULT: PASS`: build 0 warnings, urdf, both mocks, compile-only, `verify_sim.sh` (149 s), `verify_tools.sh (record, dashboard)` (34 s), scenarios `U I V K KB` pass with `S2b`, `AB` ok, every host test, hook tests, drift check, links |
| 1 and 3 sim, after the review fixes | `build_ros.sh`; `verify_tools.sh` | build exit 0; `PASS (tools: record dashboard)` (the download is now streamed; a POST with a foreign `Host` is refused). The rest of the ladder was **not** re-run after the fixes: they touch only `Dashboard.py`, the page, `RecordRun.py`'s start-up order, `package.xml` and comments |
| 4 robot | | **not run** |

A fault on the way: stopped with the launch, the first raw-subscription version died with a
`TypeError` from `deserialize_message`, because its timer ran once more after ROS had shut down.
`sample()` now returns when `rclpy.ok()` is false.

### Not verified — needs a human

- Anything on the Pi, a phone, or over Wi-Fi.
- **The dashboard's cost to the 100 Hz loop on the Pi.** Unknown; it is off by default on the robot for
  that reason. Procedure: on a stand, run the same test with and without `dashboard:=true` and compare
  the spread of cycle times in `/controller_manager/statistics/full` (record both runs with
  `record:=true`). Do this before trusting a balance run that had the dashboard on.
- The tipover tile reads log lines; if the dashboard starts after the trip it shows `armed`.
- No login: anyone on the network can press Record and download runs.
- The page's look was checked once, by eye, with made-up data; the real server's page was fetched by
  the check but not rendered.

### Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | 2 Medium, 5 Low. (M) a run's download was built whole in memory: hundreds of MB of RAM per download on the Pi, and a run still being recorded could be downloaded; (M) **the tipover tile said `armed` when the state was unknown** (no log line seen yet, or a trip logged before the dashboard started), and one wheel's re-arm cleared a trip the other still had: a safe-looking value on a safety display, finding C-2's mistake; (L) the Origin check could be passed by pointing a foreign domain name at the robot; (L) a negative `Content-Length` read without limit; (L) two races in stopping a recording, one of which could leave `ros2 bag record` running alone; (L) two comments about `ros2 run` contradicted each other; (L) three imports not declared in `package.xml` | All fixed: the tar is streamed and an open run is refused (409); the tile keeps a state per controller, shows `TRIPPED` if any is, and `no trip seen` when nothing is known; the `Host` header must be an IP address, `localhost` or the machine's own name (in the check); `0 <= length`; `ProcessLookupError` handled and `RecordRun.py` installs its signal handlers before starting the recorder; comment; three `exec_depend`s |
| 1 | `explainability-reviewer` | 5 Medium, 5 Low: the log folder default copied from `RecordRun.py`; no recipe for adding a signal, and a comment naming a list that does not exist; signs and the rad/s against m/s difference not stated; the page's script had no outline and bare pixel and millisecond numbers; `WINDOW_S` against `HISTORY_S`; the tipover phrases copied from the C++ without saying so; a repeated "not measured" claim | Fixed (the constant is imported; a recipe in the code and the guide; signs marked "not verified on the robot"; named constants and a header in the script). **Declined:** moving the request handler out of `make_handler` (it works, is commented, and class attributes set from `main()` are no easier to follow) |
| 1 | `docs-reviewer` | 2 Medium, 4 Low: `architecture/README.md` and `reproducing-results.md` still described the Flask app and a "webserver capture"; the banner's colours; ADR 0004 names "the telemetry webserver" | Fixed, except ADR 0004 (**declined**: an ADR is a record; its point, Python off the control path, still holds) |
| 2 | not run | | The fixes were re-verified as the Verification table says but **not re-reviewed** |

### Reproduce

```bash
tools/run_ladder.sh          # includes verify_tools.sh (record, dashboard)
```

## Stage E — Foxglove

Built on `b4af7ef`. **Unfinished and unreviewed**: see the HANDOFF box for what is left.

### What changed so far

| File | What and why |
| --- | --- |
| `rocko_env/webserver/launch_foxglove.py` (new), the three launch files | `foxglove:=`. Default `true` on the two robot launch files, which always started `foxglove_bridge` (a plain launch is unchanged), and `false` in the sim |
| `foxglove/rocko_balance_layout.json` (new) | A layout: four plots (pitch and target, speed and reference, wheel command and feedforwards, loop errors), the watchdog's state, the log, the raw velocity reference. **Its JSON structure is from memory of Foxglove's exported layouts and has not been opened in Foxglove** |
| `sim/check_foxglove_layout.py` (new), `verify_tools.sh` | Part 3 of the tools check: the bridge accepts a connection on port 8765, and every message path in the layout names a published topic and fields its message type really has |

### Verification so far

| Tier | Command | Result |
| --- | --- | --- |
| 1 build | `build_ros.sh` | exit 0 |
| 3 sim | `verify_tools.sh` | `PASS (tools: record dashboard foxglove)`: `all 11 message paths ... name a published topic and real fields; the bridge listens on port 8765` |
| 3 sim, by hand | the sim with `foxglove:=true`, then SIGINT to the launch | the bridge accepted a connection; nothing outlived the launch |
| the rest of the ladder | | **not run** after these changes |

Not done: the check has **not** been shown to fail on a deliberate break; no guide; no review.
Nothing can show here that Foxglove accepts or draws the layout: that needs a person with Foxglove.

**Why the run stopped:** the session's usage limit, 2026-11-17, in the middle of stage E.
