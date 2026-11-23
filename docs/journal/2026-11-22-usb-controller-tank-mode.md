# Drive tank mode from an 8BitDo controller plugged into the Pi; remove keyboard and laptop driving

- **Date / commit:** 2026-11-22, built on `bdc5545` (this file is added by the commit that adds the feature)
- **Branch:** `ai-forward`
- **Environment:** macOS laptop; dev container `rocko-env-dev` (ROS Jazzy, `ros-jazzy-joy` 3.3.0 installed into
  it for this run). **No 8BitDo controller and no robot** were available to the agent.

## Goal and success criteria

The owner asked (2026-11-22): remove keyboard driving completely; drive with an 8BitDo controller plugged into
the Raspberry Pi's USB, using the same split arcade as the Mac program; the dashboard shows the controller and
whether the robot is enabled; hold the **left** bumper to drive. Asked two questions, the owner chose: remove the
Mac paths too (the Xbox-over-Bluetooth program and `computer_joystick.py`), and "the balance mode should be
completely decoupled from the tank mode; shared constants should be put in a truly shared location".

Checkable criteria:

1. Tank mode reads the USB controller on the Pi through ROS's own `joy` package (`game_controller_node`, SDL's
   GameController layer: named, standard indices), and a new tank-only node turns it into `cmd_vel`: left stick Y
   drives, right stick X turns, deadzone with rescale, expo, wheel-saturation mixing, full motor duty at full
   stick (proposal 0014's numbers).
2. Nothing moves unless **LB** is held; after the controller goes quiet (unplugged, node dead) the command is zero
   within a fixed timeout, and LB must be seen **released** before driving resumes (the `joy` node does not reset
   buttons on a replug). Host tests prove each, red on deliberate breaks.
3. Balancing mode does not use the tank node, and the tank launch does not start `Joystick.py`. What both need
   (the controller's index layout, the shaping and mixing math, the deadman logic) lives in one module,
   `rocko_env/teleop/gamepad.py`, imported by the tank node and the dashboard.
4. The dashboard shows the controller live (sticks, triggers, buttons, LB highlighted) and an enabled /
   disabled / no-controller state, from `/joy` and the tank node's `enabled` topic. Checked in the container.
5. Keyboard and laptop driving are gone: the scripts, their tests, sim scenario `KB`, the guides, the
   keyboard-only and laptop-only branches in `Joystick.py` (precise mode, shaped mode). The two known-red checks
   (`keyboard_teleop_test.py`, `KB`) disappear with the code they tested, not by being loosened.
6. Tiers 0 to 2 pass in both modes; Tier 3 passes (minus `KB`, removed); docs and a guide for the 8BitDo.

## Plan

- New `rocko_env/rocko_env/teleop/gamepad.py` (shared) and `controllers/tank_teleop/TankTeleop.py` (tank only).
- `rocko_tank_mode.launch.py`: start `joy`'s `game_controller_node` and `TankTeleop.py`; drop `Joystick.py`.
- `Joystick.py` (balancing only): remove precise and shaped modes. Its TCP server stays as balancing's input
  (decoupled: not redesigned here), but **no sender for it is left in the repo**; that is stated, not hidden.
- Dashboard: subscribe to `/joy` and `/tank_teleop/enabled`; a controller card in the page.
- Delete `joystick/`, three host tests, scenario `KB`; update the recorder's topic list and `verify_mock.sh`.
- **Tiers:** host tests for criteria 1 to 3; Tier 1 build; Tier 2 both modes (the new nodes must start with no
  controller attached); a dashboard check that publishes a fake `/joy` in the container (criterion 4); Tier 3
  scenarios. Tier 4 is the owner's.
- **Risks:** the 8BitDo's mode (X-input vs D-input vs Switch) changes how Linux and SDL see it; signs are taken
  from the `joy` source, not seen on hardware; the `joy` package must be installed on the Pi.

## What changed

| File | What and why |
| --- | --- |
| `rocko_env/rocko_env/teleop/gamepad.py` (new) | The shared module: joy's `/joy` layout by name (LB 9, left Y 1, right X 2), its 20 Hz republish rate and the 0.25 s silence timeout, `shape()` (deadzone with rescale, expo, `min_output`; NaN and inf are 0), `mix()` (no wheel above full), `DeadmanSwitch` (hold to drive; must be seen released after start-up and after any silence) |
| `controllers/tank_teleop/TankTeleop.py` (new) | Tank mode's teleop node: `/joy` in, `cmd_vel` and `tank_teleop/enabled` out at 50 Hz; split arcade; LB; full stick = 15 m/s and 93.75 rad/s (proposal 0014); tunables as `[0, 1]` ROS parameters that fall back on a bad value |
| `bringup/launch/rocko_tank_mode.launch.py` | Starts joy's `game_controller_node` (deadzone 0, rate from `gamepad.py`) and `TankTeleop.py` instead of `Joystick.py`; launch arguments `teleop:=` (false leaves both out, for the bench test), `max_speed:=`, `max_turn:=` |
| `controllers/joystick/Joystick.py` | Balancing only: precise mode (keyboard) and shaped mode (Mac) removed; comments no longer name the deleted laptop script. Its TCP server and gamepad mapping are unchanged |
| `webserver/Dashboard.py`, `dashboard.html` | Subscribes to `/joy` and `/tank_teleop/enabled`; `/api/state` gains `controller` (by name, NaN as null) and `teleop_enabled`; the page gains a Controller card (an SVG pad drawn live, and Teleop not running / No controller / Disabled / ENABLED) |
| Deleted | `joystick/` (keyboard, Mac Xbox, gamepad sender, probe), `tools/keyboard_teleop_test.py`, `tools/xbox_teleop_test.py`, `tools/computer_joystick_heartbeat_test.py`, sim scenario `KB` (in `verify_sim.sh`, `sim/scenario_checks.py`, `tools/run_ladder.sh`), the keyboard and Xbox guides |
| `tools/tank_teleop_test.py` (new), `tools/teleop_watchdog_test.py` | Host tests (below); the watchdog test lost its precise, shaped, Mac and heartbeat checks (38 to 30) |
| `sim/check_dashboard.py`, `verify_mock.sh` | The dashboard check publishes a fake `/joy` and enable flag and requires them on the page's API; the tank mock run requires `TankTeleop.py` to have started and `game_controller_node` not to have died |
| `RecordRun.py`, `package.xml`, `CMakeLists.txt`, `DiffDriveController.py` (comment) | `/joy` and `/tank_teleop/enabled` recorded; `exec_depend` on `joy` and `sensor_msgs`; `TankTeleop.py` installed |
| Docs | [Guide](../guides/drive-with-usb-controller.md) (new), dashboard and bench-test guides (`teleop:=false`), README, guides index, next-steps (handoff; rows 4.1 and 4.7 closed, 4.5 rewritten, 4.8 new: balancing's own input), the ladder page, proposal 0014 (the ceiling now lives in `TankTeleop.py`), links from old journals into the deleted guides |

**Research** (2026-11-22): the `joy` package's `game_controller.cpp` and README on the `ros2` branch of
[ros-drivers/joystick_drivers](https://github.com/ros-drivers/joystick_drivers) (read through a web fetch): the
standard SDL layout, the -1/32767 scaling that makes up and left +1, `autorepeat_rate` 20 Hz by default, nothing
published after an unplug, buttons not reset on a replug, no display needed. On 8BitDo pads under Linux, forum
reports that X-input mode is served by the kernel's `xpad` driver and mapped by SDL by its Xbox name
([SDL forum, Pro 2 wired mapping](https://discourse.libsdl.org/t/sdl-add-linux-mapping-for-8bitdo-pro-2-wired-controller-for-xbox/42408),
[RetroPie forum, D-input vs X-input](https://retropie.org.uk/forum/post/289475)). `ros-jazzy-joy` 3.3.0 has a
`game_controller_node` executable (seen in the container).

## Verification

`./check_container_mount.sh` printed `ok` before the container tiers. Final run: `tools/run_ladder.sh --clean`
(build directories removed first), after every review fix: **RESULT: PASS**, every step green.

| Tier | Command | Result |
| --- | --- | --- |
| host (not a tier) | `python3 -I tools/tank_teleop_test.py` | **15/15**. Red on 6 deliberate breaks: no re-arm rule (3 fail), deadman on RB (6), no silence timeout (1), no desaturation (2), no NaN guard (1), wheel separation 0.30 (2). Before review it also caught the `clamp(NaN)` bug (Review rounds) |
| host | `python3 -I tools/teleop_watchdog_test.py` | **30/30** (precise, shaped, Mac and heartbeat checks removed with their code) |
| host | the other host tests, the hook tests, `sim/check_sim_config.py`, doc links | pass; links 0 broken |
| 0 static | `verify_urdf.sh` | pass |
| 1 build | `build_ros.sh` from clean | pass, **0 compiler warnings**; the new `rocko_env.teleop` package installs from scratch |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | both **pass**; in tank mode `TankTeleop.py` logged its start line and `game_controller_node` stayed up with **no controller attached**. Red on a break: with the start line renamed, `FAIL ... TankTeleop.py did not start` |
| 3 sim | `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF), scenarios (U I V K S2b AB) | pass. `KB` is gone with the keyboard; balancing's teleop path (C3, V, K through `Joystick.py`) unchanged and green |
| 3 tools | `verify_tools.sh` | pass: `fake /joy and enabled shown` (the controller by name, a NaN axis as null, the drawing in the page). Red on a break: without the `/joy` subscription, three `FAIL` lines (`controller does not show the fake /joy by name: {}` and two more) |
| page, by eye | the real `dashboard.html` served with a fake `/api/state` (a scratch script, not committed), in the built-in browser | the four states drew correctly: ENABLED (LB green), Disabled, the re-arm message, No controller (greyed). Phone width and dark mode **not checked** (the step failed and was not repeated) |
| 4 robot | | **not run** (a human only) |

**Mishaps, so nobody repeats them:** a container break test first "passed" with exit 127 because the laptop's
shell is zsh, which does not split an unquoted `$C="docker compose ..."` into words; nothing had run. Rerun with the
command written out, both breaks went red.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | No Critical/High. Medium: "reaches the wheels within 20 ms" ignores the 10 m/s² deceleration (about 1.5 s from full speed); a hand-typed int parameter crashes the node instead of falling back. Low: `verify_mock.sh` still greps for `Joystick.py` in tank mode (harmless); the new Python subpackage needs a clean build once. Plausible: the joy layout, from the reviewer's recollection of the source | Fixed: the comment now says the zero is sent within 20 ms and the wheels then slow at the limit; `dynamic_typing` on the parameters plus a test (int 0 accepted, a bool refused); a troubleshooting line for `No module named 'rocko_env.teleop'`, and the final ladder ran from a clean build. Declined: the `Joystick` grep in tank mode (harmless, keeps one loop for both modes). The deceleration limit is a guarded YAML key: recorded in proposal 0014, not changed |
| 1 | `explainability-reviewer` | Bare SVG coordinates and a second deadzone `0.1` in the page; a 150-character if/else chain; a redundant `heard` in `held`; why the wheel separation is copied; `expo` 0.6 in words; why `self.now` is a parameter; a duplicated NaN comment; `&nbsp;` alignment; a magic `5` in the timeout | All fixed (`PAD_STICKS`, `KNOB_ACTIVE_FRACTION`, a `driveStatus()` function worst case first, `MISSED_MESSAGES_BEFORE_GONE`, ...). Kept `DeadmanSwitch` as a class, as the reviewer also recommended |
| 1 | `docs-reviewer` | Stale `KB` in `verify-in-container.md`; a stale ladder row (22 checks, the laptop heartbeat); next-steps 2.5's heartbeat note; the build step's environment, branch and success line; restarting after installing `joy`; the 70 and 80 % cells of the `min_output` row off by one | All fixed except the table: the plugin **truncates** duty (`static_cast<int>`), and with the exact 119.38 rad/s the 70 % cell is 56.97, so 56 is right (the reviewer rounded with 119.4) |

The **host test found a real bug** before review: `clamp(NaN)` returned +1 (Python's `min(1.0, nan)` is 1.0), so a
NaN stick would have become full speed inside `shape()`. Fixed at the source (`shape()` returns 0 for anything not
finite); the test now checks NaN and inf.

## Not verified — needs a human

- **No 8BitDo was read.** Whether SDL recognises it in X-input mode on the Pi, its stick signs and that LB is button 9
  are taken from the `joy` source and forum reports. The guide's [first check](../guides/drive-with-usb-controller.md#first-check-what-the-pi-sees-no-driving)
  (`ros2 topic echo /joy`) is where a person confirms them **before driving**.
- **Nothing on the robot.** The stand procedure, including unplugging the controller with LB held and plugging it back in
  (the robot must not drive until LB is released), is the guide's [Drive](../guides/drive-with-usb-controller.md#drive)
  section. Abort with Ctrl-C on the Pi's launch if a wheel keeps turning after LB is released.
- **`ros-jazzy-joy` on the Pi** is not installed by anything here; the guide says `sudo apt install ros-jazzy-joy`.
- **The 100 % ceiling** (proposal 0014) is still arithmetic and still waits for the owner's confirmation; stopping time
  from full speed (the 10 m/s² limit applied when slowing) was not checked.
- **Balancing mode cannot be driven** now: `Joystick.py`'s TCP input has no sender left. Its own controller node is
  next-steps row 4.8.
- **Review:** one round per reviewer; every finding was fixed or declined with a reason above, and the clean ladder ran
  after the fixes. No second round was run.

## Reproduce

```bash
# laptop, repo root
python3 -I tools/tank_teleop_test.py
./check_container_mount.sh
tools/run_ladder.sh --clean
```

Then the guide on the Pi, on a stand, with the battery voltage and commit recorded.

Stopped because: the goal is met as far as the ladder reaches; the rest needs the 8BitDo and the robot.
