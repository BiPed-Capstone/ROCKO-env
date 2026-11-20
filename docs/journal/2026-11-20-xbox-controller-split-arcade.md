# Drive ROCKO in tank mode with an Xbox controller over Bluetooth from the Mac, split arcade

- **Date / commit:** 2026-11-20, built on `ed2d249` (this file is added by the commit that adds the feature)
- **Branch:** `ai-forward`
- **Environment:** macOS laptop (Python 3.14.4, pygame-ce 2.5.8 with SDL 2.32.10 in a scratch venv); the
  dev container `rocko-env-dev` for Tiers 0 to 2. No robot, and **the controller was paired but switched
  off** for the whole run, so no real stick value was read by the agent.

## Goal and success criteria

The owner asked for: a Python program on the MacBook that drives ROCKO **in tank mode** from an Xbox
controller over Bluetooth, **split arcade** (one stick drives, the other turns), the **full range of speed**,
**exponential mapping** on the sticks, and care with macOS joystick mapping. Nothing may change in balance mode.

Checkable criteria:

1. `joystick/xbox_teleop.py` reads the sticks by **name** (SDL GameController: left stick Y, right stick X), not
   by joystick axis number, so a different axis order on macOS cannot swap drive and turn.
2. Stick shaping on the laptop, each step a named constant: deadzone with rescale (no jump at the edge), expo
   curve, a clamp to [-1, 1]. Host test proves the numbers.
3. **Full range:** a full stick forward reaches 100 % motor duty in tank mode (computed, not measured); turning
   while at full speed keeps the turn (wheel-saturation mixing). Host test proves the mixing never asks a
   wheel for more than full.
4. **Safety:** nothing moves unless a bumper is held (deadman); losing the controller sends a stop packet and
   exits; quitting sends a stop. Host test proves each, and each test fails when the thing it guards is broken.
5. **Balance mode and the keyboard unchanged:** `Joystick.py`'s existing gamepad and precise paths give the same
   numbers as before; the new scales default to today's gamepad scales and only the tank launch file raises
   them. Host test (`tools/teleop_watchdog_test.py`) proves the paths; `verify_mock.sh` and `verify_mock.sh tank`
   still pass (Tier 2).
6. A guide a student can follow, a probe script for the first ten minutes, and docs updated.

## Plan

- **Laptop:** new `joystick/xbox_teleop.py` (pure functions for shaping and mixing, plus a send loop that
  takes a controller object so a test can fake it) and `joystick/controller_probe.py`; reuse `pack` and
  `HEARTBEAT_PERIOD_S` from `computer_joystick.py`.
- **Robot:** `Joystick.py` gets a `SHAPED_BUTTON` (14). When it is set the axes are used as sent, no second expo,
  no deadband, times two new ROS parameters. `rocko_tank_mode.launch.py` passes full-duty values; `rocko.launch.py`
  is not touched.
- **Why a flag instead of moving the expo:** `Joystick.py` already applies `expo_curve` to the gamepad path, and
  the keyboard (since `df91b9e`) relies on it. Shaping on both machines would curve the stick twice. A flag lets
  the new program own its shaping while every existing sender keeps its numbers.
- **Tiers:** host tests (not a tier) for criteria 1 to 5; Tier 0 `verify_urdf.sh`; Tier 1 build; Tier 2 both mock
  modes (the Joystick node must not crash with the new parameters). Tier 3 cannot see a controller; not run for
  this change except as part of the ladder. Tier 4 is the owner's.
- **Risks:** the speed ceiling rises from 67 % to 100 % duty in tank mode, so a first test must be on a stand
  with `--max-speed` low; macOS may stop delivering input when the Terminal is not in front (mitigated with an
  SDL hint, unverified); a Bluetooth drop that pygame does not report would freeze the last stick value (the
  robot's 1.0 s `cmd_vel_timeout` cannot help while packets keep coming).

## What changed

| File | What and why |
| --- | --- |
| `joystick/xbox_teleop.py` (new) | The Mac program. Named axes through SDL's GameController API (`pygame._sdl2.controller`), split arcade, `shape()` (deadzone with rescale, expo, optional `min_output`), `mix()` (scales forward and turn together so no wheel exceeds full), hold LB or RB to drive, Menu or Ctrl-C quits, stop and exit code 2 when the controller disappears. Sets `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1` so input continues with the Terminal behind another window (unverified on this Mac) |
| `joystick/controller_probe.py` (new) | Prints SDL's view of the controller: GUID, whether a GameController mapping exists, the named values next to the raw axes |
| `joystick/computer_joystick.py` | Defines `SHAPED_BUTTON` for the laptop side and clears it in the raw gamepad's buttons, so the old sender cannot switch the robot's mapping (`ros2-control-reviewer` round 1) |
| `rocko_env/.../joystick/Joystick.py` | `SHAPED_BUTTON` 14: axes used as sent (no second expo, no deadband) times `shaped_linear_scale_m_s` / `shaped_angular_scale`; defaults are the gamepad's -10 / -35; values beyond the full-duty 15.0 / 93.75 are refused; precise mode (15) is checked first |
| `rocko_env/.../launch/rocko_tank_mode.launch.py` | Passes -15.0 m/s and -93.75 rad/s, derived in a comment from the motor plugin's duty formula and the wheel separation. `rocko.launch.py` untouched |
| `tools/xbox_teleop_test.py` (new), `tools/teleop_watchdog_test.py`, `tools/computer_joystick_heartbeat_test.py` | Host tests, below |
| [Proposal 0014](../proposals/0014-tank-mode-full-speed-for-shaped-sticks.md) | Proposed, applied at the owner's request, for the owner to confirm: raise tank mode's stick ceiling from about 68 % to 100 % duty, and the turning-acceleration issue it exposes |
| [Guide](../guides/drive-with-xbox-controller.md), README, guides index, next-steps row 4.5 and handoff, the ladder's host-test table, the 2026-11-19 plan's status, `.gitignore` (`.venv-teleop/`) | Docs |

**Research behind the macOS choices** (web, 2026-11-20): SDL's GameController layer maps a controller to
named buttons and axes from its mapping database, while raw joystick axis numbers vary by platform, driver and
Xbox firmware ([SDL bug 4395](https://bugzilla.libsdl.org/show_bug.cgi?id=4395) on a Bluetooth layout change);
on macOS SDL can reach the controller through its HIDAPI driver or Apple's Game Controller (MFi) framework, and
`SDL_JOYSTICK_HIDAPI=0` / `SDL_JOYSTICK_MFI=0` are the documented ways to pick the other one
([SDL forum](https://discourse.libsdl.org/t/sdl-isgamecontroller-returning-false-for-xbox-one-wired-controller-on-macos/22790));
controller input is dropped for a background app unless
[`SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS`](https://wiki.libsdl.org/SDL2/SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS)
is 1. Plain `pygame` had no Python 3.14 wheels; `pygame-ce` 2.5.8 installed from a wheel and was used.

## Verification

`./check_container_mount.sh` printed `ok    the container sees this checkout` before the container tiers.

| Tier | Command | Result |
| --- | --- | --- |
| host (not a tier) | `python3 -I tools/xbox_teleop_test.py` | **17/17** pass. Red on 4 deliberate breaks: deadman removed (2 fail), no stop on controller loss (1), no desaturation in `mix()` (3), deadzone without rescale (2) |
| host | `python3 -I tools/teleop_watchdog_test.py` | **38/38** pass (was 32; 6 new shaped-mode checks, one gamepad check rewritten because button 14 now has a meaning). Red on 4 breaks: `SHAPED_BUTTON` 13 on the laptop, the Pi ignoring the flag, the balance default raised to -15, the scale bound removed |
| host | `python3 -I tools/computer_joystick_heartbeat_test.py` | **6/6** (one new). Red when the button-14 clear is removed |
| host | `python3 -I tools/keyboard_teleop_test.py` | **4/5, known red** since `df91b9e` (the precise-button check), unchanged by this change ([2026-11-19 journal](2026-11-19-first-robot-run-tank-mode.md#known-red-on-purpose-left-red)) |
| 0 static | `verify_urdf.sh`, hook tests, doc links, sim config drift (in `tools/run_ladder.sh`) | pass; links 880, 0 broken |
| 1 build | `build_ros.sh` | pass, **0 compiler warnings**. Says nothing about the motor plugin (not touched; its compile-only check also passed) |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | both **pass** after the round-1 fixes (`tools/run_ladder.sh --no-sim`) |
| 2 mock, by hand | `ros2 launch rocko_env rocko_tank_mode.launch.py use_mock_hardware:=true dashboard:=false`, then `ros2 param get /joystick shaped_linear_scale_m_s` / `shaped_angular_scale` | `-15.0` and `-93.75`; the node logged `Shaped-axis scales: linear -15.0, angular -93.75`. The same readback for `rocko.launch.py` did not complete (the shell hung stopping the first launch) and is **not run**; that the balancing launch passes no shaped parameter is the host test's text check |
| 3 sim | `tools/run_ladder.sh` (full), **before** the round-1 fixes | `verify_sim.sh` S1 S2a H4 C3 C2 H6 FF pass; `verify_tools.sh` pass; scenarios U I V K pass, S2b and AB behave as designed, **KB fails, known red**, with the same three speeds as 2026-11-19 (0.5545, 1.136, 1.7715 m/s). The round-1 fixes (a scale bound, a laptop-side button mask) do not touch the gamepad path KB drives; Tier 3 was not rerun after them. No sim check sees a controller |
| laptop, real pygame-ce | `xbox_teleop.py --help`; `--max-speed 1.5`; a run with the controller off; `controller_probe.py` off; `stick_reader()` against a fake pad with pygame's real constants and a posted `CONTROLLERDEVICEREMOVED` | help prints; 1.5 refused; `No controller found...` after 5.6 s, exit 1; the probe says `No joystick...`; the reader maps -32768 to -1.0 and returns `attached=False` after the removal event |
| 4 robot | | **not run** (a human only) |

**Mishaps, so nobody repeats them:** `python3 -I` ignores `PYTHONDONTWRITEBYTECODE`, so during the break
tests a stale `.pyc` of a broken `xbox_teleop.py` (same size, same second) was reused and showed a false red;
deleting `joystick/__pycache__` fixed it. And `pkill -9 -f "ros2 launch"` inside `bash -c` matched its own
command line and killed the shell (exit 137); a leftover `foxglove_bridge` was then stopped by PID.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `ros2-control-reviewer` | **High:** the launch file raises tank mode's real top speed (68 % to 100 % duty), a limit in spirit that the guard cannot see; `shaped_scale` had no upper bound. **Medium:** `computer_joystick.py` forwards raw buttons, so a raw button 14 would switch the mapping. **Low:** `angular.z.max_acceleration` 1000 means a full turn reaches full duty in about 0.1 s | High: the owner asked for full speed in chat, so it is applied and recorded as [proposal 0014], **Proposed** until the owner confirms the specific ceiling (docs-reviewer round 2)(../proposals/0014-tank-mode-full-speed-for-shaped-sticks.md), and `Joystick.py` now refuses scales beyond the full-duty values (test red on a break). Medium: fixed, button 14 cleared in `computer_joystick.py` (test red on a break). Low: a guarded YAML key, not changed; in proposal 0014 and the guide (`--max-turn` low first) |
| 1 | `explainability-reviewer` | The sign chain spread over three files; `left` really a turn; `Settings` duplicates defaults; bare `0.1`/`5.0`; `shape()` divides by zero at deadzone 1; `mix()` step order unstated; wheel separation copied; no source line for the duty formula; a renamed angular parameter; whitespace | Fixed: a sign-chain comment in `command_from`, `turn_left`, a dataclass, `STATUS_PERIOD_S` / `CONNECT_TIMEOUT_S`, the deadzone precondition, numbered steps in `mix()`, the line reference, a pointer to the test that catches separation drift. Declined: renaming `shaped_angular_scale` (its unit is mode-dependent, which the comment says); a new grep test for button 14 (the existing test already compares the two files' constants); the whitespace (it matches the file's two-blank-line style) |
| 2 | `ros2-control-reviewer` | CLEAN. Note: the launch value sits exactly on the cap, so a changed separation would be refused and fall back to -35 quietly; check the node's `Shaped-axis scales` line, not `ros2 param get` | Noted; the hand check above did read the log line |
| 2 | `docs-reviewer` | 0014 called Accepted on a feature request; one count left in the ladder's command block; the keyboard row's "5 checks" (pre-existing) | Fixed the first two: 0014 is Proposed (applied, owner to confirm). The third predates this change and is recorded as known red elsewhere |
| 1 | `docs-reviewer` | "Wheels stop at once" is stronger than the code (tank mode slows at 10 m/s²); evidence claimed before the journal held it; a stale cross-reference; counts written twice on the ladder page; which user builds on the Pi; heading still says "stage 4" | Fixed all but the heading: renaming it broke four links into it, so it was kept |

## Not verified — needs a human

- **No real controller value was read.** The controller is paired with this Mac but was switched off
  (`system_profiler SPBluetoothDataType`: "Not Connected"). Whether SDL 2.32.10 has a GameController mapping for
  it over Bluetooth, which driver (HIDAPI or MFi) it uses, the resting offsets, and whether input continues with
  the Terminal in the background are all for the probe: [guide, *First*](../guides/drive-with-xbox-controller.md#first-look-at-what-the-mac-sees-ten-minutes-no-robot).
- **Nothing on the robot.** The stand procedure, with the controller switched off mid-drive and the time to stop
  recorded, is the guide's [*Drive*](../guides/drive-with-xbox-controller.md#drive) section. Abort with Ctrl-C on
  the Mac or by stopping the Pi's launch if a wheel keeps turning after the bumper is released.
- **The 100 % ceiling** (proposal 0014) is arithmetic from the plugin's duty formula, which itself uses a disputed
  wheel radius and an integer-division `MAX_RAD_PER_SEC` (L-7). Whether 15 m/s is what the motors do is unknown.
- **Stopping time from speed.** Whether `diff_drive_controller` applies the 10 m/s² limit when slowing down was not
  checked.
- **A Bluetooth drop that SDL does not report** would leave the last values, bumper included, in place. Not
  testable without the controller.
- **Balancing mode** with this program: not tried and not intended.

## Reproduce

```bash
# laptop, repo root
python3 -I tools/xbox_teleop_test.py
python3 -I tools/teleop_watchdog_test.py
python3 -I tools/computer_joystick_heartbeat_test.py
./check_container_mount.sh
tools/run_ladder.sh
```

Then the guide, on a stand, with the battery voltage and commit recorded.

Stopped because: the goal is met as far as the ladder reaches; the rest needs the controller switched on and the robot.
