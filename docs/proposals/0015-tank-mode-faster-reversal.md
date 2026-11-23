# Proposal 0015 — make tank mode reverse faster: raise its deceleration (and possibly acceleration) limits

- **Status:** Open
- **Gate:** hardware limits (`linear.x` acceleration keys in `bringup/config/rocko_tank_mode_controllers.yaml`;
  CLAUDE.md section 3)
- **Raised:** 2026-11-22, commit `784022a`, run journal:
  [2026-11-22-tank-reversal-lag.md](../journal/2026-11-22-tank-reversal-lag.md)

## The problem, as the owner saw it

Driving tank mode with the USB controller (robot on blocks, `max_speed:=0.4`), full forward then straight to full
reverse is "very slow to switch directions ... as if it has to wind down and then wind up in reverse", which
reads to the driver as lag.

## What causes it (measured)

Measured on the robot on 2026-11-22 (owner present, robot on blocks, commit `784022a`, `pi/launch.sh tank
max_speed:=0.4 max_turn:=0.3`; battery voltage **not recorded**), from `/diffbot_base_controller/cmd_vel`
(the stick), `cmd_vel_out` (the rate-limited command) and the left encoder's raw position:

| After the stick flips +6 to -6 m/s | Command (`cmd_vel_out`) | Wheel (encoder position) |
| --- | --- | --- |
| crosses zero | 0.56 s | 0.55 s |
| reaches full reverse | 1.2 s (90 % at 1.09 s) | about 1.1 s |
| barely moving (\|v\| < 0.15 units/s) | while the command passes +0.6 to -2.4 m/s | 0.5 to 0.9 s |

1. **The controller's acceleration limit is the delay.** `TankTeleop.py` sends the new target in one step;
   `diff_drive_controller` ramps its output at `linear.x.max_acceleration: 10.0` m/s², 1.0 m/s every 0.1 s:
   12 m/s of change takes 1.2 s. At full speed (`max_speed:=1.0`, 15 m/s) it is **3.0 s**.
2. **The wheels follow the ramp closely** (zero crossing within 0.01 s of the command's, full reverse no later), on
   the way down too: no coasting on inertia. Nothing in this data points at the motors or the driver.
3. **About 0.4 s of each reversal is a dead spot**: the ramp spends that long below the motors' 21 to 28 % breakaway
   duty ([proposal 0013](0013-tank-mode-motor-breakaway.md)), where they only hum.

Not the cause: the joystick path (a released LB became a zero command in 11 to 19 ms), the 50 Hz control rate, and
the encoder node's velocity filter (tank mode drives open loop and never uses it).

**A setting in the YAML does nothing:** `linear.x.min_acceleration: -10.0`. On the mock
(`diff_drive_controller` 4.42.1) changing it changed nothing (below), and the installed parameter description calls it
"deprecated, use max_deceleration"; slowing down is limited by
`max_acceleration` unless the newer `linear.x.max_deceleration` is set.

## The options, measured on the mock

`tools/measure_reversal_ramp.sh` steps the command from +V to -V in the container on the mock and times
`cmd_vel_out`. The ramp is pure software, so the mock measures it exactly (as-is it gave 0.60 s and 1.14 s, matching
the robot's 0.56 s and 1.09 s). The limits were set with `ros2 param set` and the controller reconfigured; the YAML
was not edited. The container's `diff_drive_controller` is 4.42.1, the Pi's 4.43.1: the parameter names and
signs are the same in both (checked in the Pi's installed headers), the timings are from 4.42.1.

| Option | YAML (linear.x) | Reversal at 6 m/s (90 %) | at 15 m/s (90 %) | Starts from rest |
| --- | --- | --- | --- | --- |
| As is | `max_acceleration: 10.0` | 1.14 s | 2.86 s | 0 to 6 m/s in 0.6 s |
| **A: faster slowing only** | add `max_deceleration: -30.0`, `max_deceleration_reverse: 30.0` | 0.72 s | 1.82 s | unchanged |
| **B: faster both ways** | `max_acceleration: 30.0` and A's two lines (measured exactly so) | 0.38 s | 0.96 s | 0 to 6 m/s in 0.2 s |
| C: nothing | | 1.14 s | 2.86 s | |

In every option, delete the ineffective `linear.x.min_acceleration: -10.0` line so nobody tunes it. Signs are the
controller's own: `max_deceleration` and `max_acceleration_reverse` are 0 or negative, `max_deceleration_reverse` 0 or
positive (from its parameter checks).

The dead spot shrinks with a steeper ramp (at 30 m/s² the band of ±3.9 m/s is crossed in about 0.25 s instead of
0.8 s), but only proposal 0013 (a stiction duty in the motor plugin) removes it.

**Recommendation:** B for tank mode, because the delay the driver feels is the whole ramp, not just the slowing half.
A first if a jerky start on the floor turns out to be a problem. Angular limits are already effectively off
(1000 rad/s²); nothing to change there.

## Risks

- A faster reversal is harder on the gearboxes and the motors' current (reversing a spinning motor briefly draws
  more current); nobody has measured current or temperature on this robot.
- On the floor, a 30 m/s² change at speed can make the robot lurch or tip forward/back. Test on blocks first, then on
  the floor at `max_speed:=0.4`.

## Test on the robot (a human)

1. Edit the YAML lines of the chosen option, rebuild (`./build_ros.sh` on the Pi), start
   `pi/launch.sh tank max_speed:=0.4 max_turn:=1.0` with the robot **on blocks**.
2. Full forward, then full reverse, a few times. Success: the direction changes in about the table's time; the wheels
   do not slip on their hubs or bang; nothing smells hot. Abort (Ctrl-C on the launch) on a bang, a stall that
   continues, or heat.
3. Then the floor, with space around it, still at `max_speed:=0.4`; raise it in steps.
4. Record the battery voltage, the commit and what you saw as [reproducing results](../guides/reproducing-results.md)
   asks.
