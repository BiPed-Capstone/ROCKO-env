# Proposal 0014 — let a full stick reach 100 % motor duty in tank mode

- **Status:** **Proposed, and applied** at the owner's request. On 2026-11-20 the owner asked for the Xbox
  controller program with "the full range of speed"; that is a feature request, not a sign-off on this
  specific ceiling, so **the owner is asked to confirm 15 m/s / 100 % duty** (set this to Accepted, or reverse it
  as below). **Not run on a robot.** **Moved 2026-11-22:** the Mac program is gone; the same ceiling now lives
  in tank mode's own teleop node, `TankTeleop.py` (the USB controller on the Pi). The decision is unchanged.
- **Gate:** hardware limits, in spirit. No guarded YAML key changed, but the change raises the real top speed
  of tank mode outside the YAML (first a launch file, now `TankTeleop.py`), which the hook cannot see (the `ros2-control-reviewer` raised this,
  2026-11-20). It is recorded here so the decision is visible and can be reversed in one place.
- **Raised:** 2026-11-20, built on `ed2d249`, run journal:
  [2026-11-20-xbox-controller-split-arcade.md](../journal/2026-11-20-xbox-controller-split-arcade.md)

## The change

As it stands since 2026-11-22:

- `controllers/tank_teleop/TankTeleop.py`: `FULL_WHEEL_SPEED_M_S` = 15.0 and `FULL_TURN_RAD_S` = 15.0 / (0.32 / 2) =
  93.75 are what a full command means. A full stick forward is 15 m/s; a full turn alone is 93.75 rad/s.
- `rocko.launch.py` and balancing's `Joystick.py`: **not involved**; balancing keeps the gamepad scales (-10, -35).

From 2026-11-20 to 2026-11-22 the same numbers were shaped-axes parameters that `rocko_tank_mode.launch.py`
passed to `Joystick.py` for the Mac program (bounded there at the same values); both are removed.

| Full stick, tank mode | Before (gamepad scale) | After (this proposal) |
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
  Until a human decides, start with `max_turn:=` well below 1 (the guide says so).
- **Stopping from speed.** Letting go of the bumper sends zero at once, but the wheels may take up to about
  1.5 s to slow from full speed if the controller applies the same 10 m/s² limit when slowing (not checked).

## Evidence

Host tests only (`tools/tank_teleop_test.py`: full stick = 15 m/s and 93.75 rad/s, the mixing keeps both wheels
within full speed, the wheel separation matches the YAML; red on deliberate breaks). Tier 2: tank mode starts on the
mock with `TankTeleop.py` up. Nothing on a robot. Journals:
[2026-11-20](../journal/2026-11-20-xbox-controller-split-arcade.md),
[2026-11-22](../journal/2026-11-22-usb-controller-tank-mode.md).

## Test on the robot (a human)

On a stand, wheels off the ground, battery voltage recorded:
[the guide's "Drive" section](../guides/drive-with-usb-controller.md#drive), starting at `max_speed:=0.4
max_turn:=0.3` and raising in steps. Abort (stop the Pi's launch with Ctrl-C) if a wheel keeps
turning after LB is released, or if the motors or driver get hot. Then the floor, with space around it.

## To reverse it

Lower `FULL_WHEEL_SPEED_M_S` in `TankTeleop.py` (10.0 gives the old 68 %); `FULL_TURN_RAD_S` follows from it.
