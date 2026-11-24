# A supervisor node owns tank/balance mode and enable, switched from the controller or the dashboard

- **Date / commit:** 2026-11-23, built on `f3ea0c5` (the commit that adds this file holds the change)
- **Branch:** ai-forward
- **Environment:** laptop (macOS) + the `rocko-env-dev` container (ROS 2 Jazzy); no Pi, no robot

## Goal and success criteria

The owner asked, on 2026-11-23: switch tank ⇄ balance from the dashboard or by pressing LT+RT+RB together; a
**disabled** state, toggled by LB+RB or dashboard Enable/Disable buttons, the same in both modes; mode changes only
while disabled; the dashboard always shows the mode and the state of the running nodes, especially during a change;
the ROS service starts at boot in tank mode; and an architecture a robotics engineer would build, with room for
autonomous modes, app versus controller driving and optional sensors. Balance mode also gets its gamepad input
(row 4.8). The owner's answers to the agent's questions: a mode change restarts the stack; combos fire instantly;
enable is a toggle that replaces tank mode's hold-LB deadman.

Design: [robot-supervisor.md](../architecture/robot-supervisor.md), [ADR 0006](../decisions/0006-robot-supervisor.md).

Checkable criteria:
1. Host test: the state machine obeys every rule in the design (enable only when ready; mode change only when
   disabled; disable always; auto-disable on controller silence and on a lost node; balance enable refused outside
   the pitch band or when the pitch is unknown), and both combos fire once on the rising edge, never on reconnect.
2. Host tests: both teleop nodes send zero unless enabled in their own mode, the controller is heard, and the sticks
   were re-centred after enable.
3. Tier 1: both packages build in the container, with the new messages and services.
4. Tier 2 (mock): `rocko_robot.launch.py use_mock_hardware:=true` boots into tank, disabled, phase ready, with
   `diffbot_base_controller` inactive; enable activates it; a mode change while enabled is refused; disable then
   change to balance reaches ready with the four cascade controllers inactive; enable activates them; the LB+RB and
   LT+RT+RB combos on a published `/joy` do the same; after a disable the mock's wheel velocities read 0; a killed
   stack node gives `fault` and disabled.
5. The existing checks still pass: `verify_urdf.sh`, `verify_mock.sh`, `verify_mock.sh tank`, the hook tests,
   `tools/pi_scripts_test.sh`.

## Plan

Three implementers in parallel on separate files (Sonnet), then integration and Tier 2 by the orchestrator:
(A) `rocko_interfaces` messages and services, `rocko_env/supervisor/` (state machine, mode table, node),
`rocko_robot.launch.py`, CMake; (B) `TankTeleop.py` without the deadman, new `BalanceTeleop.py`, `supervised:=true`
in both mode launch files; (C) the dashboard's state card, buttons and node table, `pi/rocko.service`,
`pi/launch.sh robot`, `pi/install_boot_service.sh robot`. The shared combo logic (`ComboDetector`, `CentreRearm`) went
into `teleop/gamepad.py` first so all three build on one copy.

Risks: a stack that will not stop (C-1), handled with SIGKILL after a timeout and reported as a fault (H-7); the
spawner's `--inactive` with `--activate-as-group`; no §6 critical-path file is meant to change, so `realtime-auditor`
is not mandatory unless one does.

## What changed

- `rocko_interfaces/msg/RobotState.msg`, `NodeStatus.msg`, `ControllerStatus.msg`, `srv/SetMode.srv`, `srv/SetEnabled.srv`:
  the state topic and the two services ([design](../architecture/robot-supervisor.md#the-ros-interfaces)).
- `rocko_env/rocko_env/supervisor/state_machine.py` (every rule, plain Python), `modes.py` (the one mode table),
  `Supervisor.py` (the node: starts and stops each mode's launch as a child process group, activates and deactivates
  controllers through `controller_manager`, reads `/joy` combos and the pitch, publishes `/rocko/state`).
- `bringup/launch/rocko_robot.launch.py` (new, the boot launch: supervisor, joy node, dashboard, Foxglove).
- `rocko_tank_mode.launch.py`, `rocko.launch.py`: `supervised:=true` leaves out what the base launch runs and spawns the
  motion controllers `--inactive`; supervised balance also spawns `joint_state_broadcaster`, so the supervisor can read
  pitch (its cost at 100 Hz is unmeasured: row 4.15). **Standalone `rocko.launch.py` changed on purpose** (row 4.8):
  it no longer starts `Joystick.py` (the TCP laptop path, which had no sender left) but the joy node and
  `BalanceTeleop.py`. The sim launch still starts `Joystick.py`.
- `controllers/tank_teleop/TankTeleop.py`: the hold-LB deadman is gone (owner's decision, ADR 0006); it drives only
  when `/rocko/state` says enabled in tank mode, `/joy` is fresh and the sticks re-centred after enable.
- `controllers/balance_teleop/BalanceTeleop.py` (new): the same gate for balance mode, publishing on the topic
  `DiffDriveController.py` reads. Default caps 0.5 m/s forward and 0.35 turn, chosen by the agent as low starting
  points; the implementer first copied the old laptop scale, which would have allowed 3 m/s.
- `teleop/gamepad.py`: `ComboDetector`, `CentreRearm`, `pressed_inputs`, trigger helpers; `DeadmanSwitch` deleted.
- `webserver/Dashboard.py`, `dashboard.html`: state card, Enable/Disable/Tank/Balance buttons, nodes and controllers
  table; it now calls the supervisor's two services (still no login).
- `pi/rocko.service`, `pi/install_boot_service.sh robot`, `pi/launch.sh robot` and its guard.
- Checks: `verify_supervisor.sh` + `tools/supervisor_flow_check.py` (Tier 2, in `tools/run_ladder.sh`),
  `tools/supervisor_state_machine_test.py`, `tools/balance_teleop_test.py`; updated `tools/tank_teleop_test.py`,
  `tools/pi_scripts_test.sh`, `sim/check_dashboard.py` (allows the dashboard's two service clients), `verify_mock.sh`
  (the tank teleop's new start-up line).
- Docs: the architecture page, ADR 0006, the three guides, the ladder, next-steps (handoff box, rows 4.5, 4.8,
  4.11 to 4.15), README.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `python3 -I tools/supervisor_state_machine_test.py` | PASS 23 tests. Red on deliberate breaks: the pitch band removed; the "disable before changing mode" rule removed; balance's "stay enabled on lost input" branch disabled (each restored) |
| host | `tools/tank_teleop_test.py`, `tools/balance_teleop_test.py` | 10/10, 6/6. Red on three broken gates (mode check, centre re-arm, state age), restored |
| host | `bash tools/pi_scripts_test.sh` | 29 passed. Red on 4 deliberate breaks (6 failures), restored |
| 0 static | `verify_urdf.sh`, hook tests, doc link scan | pass (ladder) |
| 1 build | `build_ros.sh` | pass, 0 compiler warnings (ladder) |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | pass |
| 2 mock | `verify_supervisor.sh` (34 checks: boot, enable, wheels move when enabled and read 0 within 0.07 s of a disable, refusals, tank⇄balance, balance enable, both combos, auto-disable on `/joy` silence in tank, fault on a killed `controller_manager`, balance stays enabled when its teleop node is killed, clean shutdown) | 3 of 3 after round 1 fixes; pass in the second ladder; after the round 2 fixes, rebuilt and run twice more: 34 passed, 0 failed, both times. Red on a deliberate break (unanswered `controller_manager` treated as nothing), restored. A mode switch took 3 to 5 s on the mock (the laptop, not the Pi) |
| 3 sim | `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF) | pass (both ladders) |
| 3 sim | `verify_sim.sh scenarios` | first ladder pass; second ladder **FAIL: K overshoot 66.4 %**; then `tools/repeat_check.sh "scenarios K" 5`: pass 5, fail 0. K's inputs (sim launch, `DiffDriveController.py`, `Joystick.py`) are unchanged by this work; earlier harness stalls are recorded in the ladder page. Cause of the one failure **not found** |
| 3 sim | `verify_tools.sh` | first ladder **FAIL** (a real bug: the dashboard took a lock it already held; fixed, red when restored); second ladder **FAIL: only 38 history points** (40 needed); then 3 runs pass with 116, 109 and 66 points, and one more after the round 2 fixes with 110. `sample()` is unchanged; the spread looks load-driven, but the cause is **not found** and this stays an open red |

Warnings read: the build has none; launch logs show the known `FIFO RT scheduling` warning in the container and
`DiffDriveController.py` tracebacks on SIGINT (pre-existing, not ours).

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | realtime-auditor | 3 high: an unanswered `controller_manager` read as "missing" disabled balance (a fall); a failed disable was never retried while the state said disabled; a crashed launch left its process group; plus no automatic kill of a hung stack; zero-on-deactivate can fail silently | first four fixed (unknown vs missing, confirmed and retried disables, group kill). The last is pre-existing in a §6 file: filed as row 4.14, not fixed here |
| 1 | ros2-control-reviewer | failed disable (same as above); standalone balance launch changed; stale CMake guard; failed-enable message | fixed; the launch change is intended and stated above |
| 1 | explainability-reviewer | dead `DeadmanSwitch`; finding IDs in user messages; bare timeouts; dense lines | all fixed |
| 1 | agent's own | auto-disable on `/joy` silence would have dropped a balancing robot (C-3) | fixed before review: balance holds zero velocity instead |
| 2 | realtime-auditor, ros2-control-reviewer | a crashed launch's reason was overwritten by a false "could not confirm"; the disable-confirm window (3 s) was shorter than the switch timeout (5 s); pgid reuse window; is killing the group of a crashed launch right in balance | fixed (order, 6 s window); pgid gap accepted with a comment; the group kill kept on purpose and commented (an orphaned stack nothing supervises is worse) |
| 2 | docs-reviewer | stale ladder row for the tank teleop test; the kill rule incomplete for a disabled balance stack | fixed |

Round 3 was not run: round 2's confirmed findings were all low severity and are fixed above; the final Tier 2 runs
and host tests are after those fixes. Two open reds are stated in Verification: scenario K (one failure in 6 runs)
and `verify_tools.sh`'s history count (one failure in 5 runs), both without a found cause.

## Not verified — needs a human

- **Nothing here has run on the Pi or the robot.** Not the boot service, the joy node with the 8BitDo, the trigger
  reading (|axis| > 0.5 is inferred), the dashboard on a phone, the CPU cost of the supervisor and of
  `joint_state_broadcaster` beside the 100 Hz loop (row 4.15), or that a deactivate leaves the real motors at 0 %.
- **Balancing has never run on the robot.** This change only makes balance mode reachable and gives it a gamepad input;
  the balancing queue in next-steps (handoff box) is unchanged. Do not enable balance mode before that queue's steps.
- Known gap: `rocko_robot.launch.py` does not pass `max_speed:=`/`max_turn:=` through to the mode stacks, so a stand test
  under the supervisor cannot lower the caps (the guide says so).
- **Owner's procedure, tank mode only, wheels off the ground:**
  1. On the Pi, in your shell: `sudo apt install ros-jazzy-joy` if not already; `git pull`; rebuild as in
     [run-on-the-pi.md](../guides/run-on-the-pi.md).
  2. `pi/install_boot_service.sh robot` (it removes the tank and dashboard services), then `sudo reboot`.
  3. Open the dashboard: it should show **TANK, DISABLED, ready** and every node and controller green.
  4. Press **LB+RB**: ENABLED. Sticks centred, then a small push: the wheels turn the right way. **LB+RB** again:
     they stop at once. Abort (lift the power) if they do not stop.
  5. Unplug the controller while enabled: the dashboard should go DISABLED within a second.
  6. While disabled press **LT+RT+RB**: the dashboard shows the stack stopping and starting, then **BALANCE, DISABLED,
     ready**. Do **not** enable it. **LT+RT+RB** again returns to tank. Try the dashboard buttons for the same.
  7. Record what each step did in a journal entry, including any step that failed.

## On the robot, 2026-11-23 (owner present, agent over SSH with the owner's permission)

- **First start failed:** the owner installed `rocko.service` after `git pull` without rebuilding. The log:
  `file 'rocko_robot.launch.py' was not found in the share directory`, three restarts, then systemd gave up.
  Fixed on the Pi with `./build_ros.sh` (both packages, the motor plugin included: the first Pi build of this code).
  `pi/install_boot_service.sh robot` now refuses to install in an unbuilt checkout and says to build (host test, red
  when the check is removed).
- **Then it ran:** `rocko.service` active; `/api/state` on the Pi: tank, disabled, ready, `joint_state_broadcaster`
  active, `diffbot_base_controller` inactive, every expected node alive; the page answers HTTP 200. The 8BitDo was found
  by the joy node. From the log, the owner then enabled and disabled tank mode with LB+RB four times (each switch
  logged by `controller_manager` within about 10 ms) and switched to balance with LT+RT+RB; the tank stack stopped
  cleanly in about 0.5 s. What the wheels did is the owner's to report.
- **Log flood found and fixed:** while disabled, `TankTeleop.py` sent a zero command 50 times a second to the inactive
  `diff_drive_controller`, which logged a warning for each (1651 lines in a minute). Both teleop nodes now send nothing
  while disabled. After the fix on the Pi: 0 such lines in 15 s.
- **Balance mode could not start, and the supervisor handled it:** `ICM20948.py` exited at start-up
  (`ModuleNotFoundError: No module named 'gpiod'`, needed by Adafruit's `board` on a Pi 5), and the venv also lacks
  `adafruit_bno055`. The C++ IMU plugin then waited forever for the node's service (finding C-1's unbounded wait), so
  `controller_manager` never answered; the cascade was never loaded and the motors were never driven in balance mode.
  The robot was disabled, so the supervisor reported `fault` and killed the stack, as designed. Not fixed: the two
  packages are not installed (owner's decision; README's dependency table now lists them).
