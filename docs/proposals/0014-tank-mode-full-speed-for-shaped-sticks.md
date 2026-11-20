# Proposal 0014 — let a shaped stick reach 100 % motor duty in tank mode

- **Status:** **Proposed, and applied** at the owner's request. On 2026-11-20 the owner asked for the Xbox
  controller program with "the full range of speed"; that is a feature request, not a sign-off on this
  specific ceiling, so **the owner is asked to confirm 15 m/s / 100 % duty** (set this to Accepted, or reverse it
  as below). **Not run on a robot.**
- **Gate:** hardware limits, in spirit. No guarded YAML key changed, but the change raises the real top speed
  of tank mode from a launch file, which the hook cannot see (the `ros2-control-reviewer` raised this,
  2026-11-20). It is recorded here so the decision is visible and can be reversed in one place.
- **Raised:** 2026-11-20, built on `ed2d249`, run journal:
  [2026-11-20-xbox-controller-split-arcade.md](../journal/2026-11-20-xbox-controller-split-arcade.md)

## The change

- `rocko_tank_mode.launch.py`, the `joystick` node: passes `shaped_linear_scale_m_s` = -15.0 and
  `shaped_angular_scale` = -93.75. Before, there was no such path, and a full gamepad stick asked for 10 m/s
  and 35 rad/s.
- `Joystick.py`: the new shaped-axes path (button byte 14, used only by `joystick/xbox_teleop.py`) scales by
  those parameters. Its defaults are the old gamepad scales (-10, -35), and values beyond the full-duty
  ones (`MAX_SHAPED_LINEAR_SCALE_M_S` 15.0, `MAX_SHAPED_ANGULAR_SCALE` 93.75) are refused.
- `rocko.launch.py` (balancing): **unchanged**; it passes no shaped parameter.

| Full stick, tank mode | Before (gamepad scale) | After (shaped, this proposal) |
| --- | --- | --- |
| Forward | 10 m/s, about 68 % duty | 15 m/s, 100 % duty |
| Turn alone, each wheel | 35 x 0.16 = 5.6 m/s, about 39 % duty | 15 m/s, 100 % duty |

Duty = wheel rad/s / 119.4 x 100 + 2, clamped at 100 (`Motor12VoltQuadEncoder.cpp:231`); wheel rad/s = m/s /
0.127. Arithmetic from the code, not measured.

## Why

The owner asked for the full speed range on the controller. Below about 21 to 28 % duty the motors only hum
([proposal 0013](0013-tank-mode-motor-breakaway.md)), so with the old 68 % ceiling and an expo curve most of
the stick's travel would be spent humming or barely moving. The YAML's velocity limits (20 m/s, 1000 rad/s)
were already above what the motors can do, so they never limited tank mode and still do not.

## What this does not change, and what it makes matter

- **Acceleration.** `rocko_tank_mode_controllers.yaml` limits `linear.x` to 10 m/s² (about 1.5 s from 0 to
  15 m/s) but `angular.z` to 1000 rad/s², so a full turn reaches 100 % duty on each wheel in about 0.1 s. That
  value is old; this change is the first to make it matter. Changing it is a guarded YAML key, not done here.
  Until a human decides, start with `--max-turn` well below 1 (the guide says so).
- **Stopping from speed.** Letting go of the bumper sends zero at once, but the wheels may take up to about
  1.5 s to slow from full speed if the controller applies the same 10 m/s² limit when slowing (not checked).

## Evidence

Host tests only (`tools/teleop_watchdog_test.py`: the shaped mapping, the bound, the launch file's numbers
against the YAML and against `Joystick.py`'s maximums; red on deliberate breaks). Tier 2: the node starts on the
mock with the new parameters (see the journal). Nothing on a robot.

## Test on the robot (a human)

On a stand, wheels off the ground, battery voltage recorded:
[the guide's "Drive" section](../guides/drive-with-xbox-controller.md#drive), starting at `--max-speed 0.4
--max-turn 0.3` and raising in steps. Abort (Ctrl-C on the Mac, or stop the Pi's launch) if a wheel keeps
turning after the bumper is released, or if the motors or driver get hot. Then the floor, with space around it.

## To reverse it

Delete the `parameters=[...]` block from the `joystick` node in `rocko_tank_mode.launch.py`. The shaped path
then uses the old gamepad scales.
