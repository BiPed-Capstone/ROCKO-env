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
> | A | Ramp the drive commands in `DiffDriveController.py`; sim scenario `K` | not started |
> | B | Keyboard driving from the laptop (`joystick/keyboard_teleop.py`) | not started |
> | C | Bag recording on the Pi (`record:=true`), metadata, `tools/fetch_logs.sh` | not started |
> | D | Web dashboard (replaces the Flask stub), with Record/Stop | not started |
> | E | Foxglove layout and guides | not started |
>
> **Next action:** start stage A.
>
> **Uncommitted work when this was written:** none.
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

Not started.

## Stage B — keyboard driving

Not started.

## Stage C — logging

Not started.

## Stage D — web dashboard

Not started.

## Stage E — Foxglove

Not started.
