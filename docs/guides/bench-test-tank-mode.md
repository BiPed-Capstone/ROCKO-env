# Bench-test tank mode: do the wheels turn, and at what command?

**Who this is for:** the person at the robot the first time `rocko_tank_mode.launch.py` runs, or
anyone whose wheels hum and do not turn.

**What has been checked:** the owner ran this test on 2026-11-19; the breakaway of each wheel is marked
in the table below ([journal](../journal/2026-11-19-first-robot-run-tank-mode.md)). The duty numbers below come from
reading the code (worked out so you can check them). Every step that touches the
robot is yours: an agent on this branch does not launch anything on the Pi
([CLAUDE.md](../../CLAUDE.md) section 3).

**Why it exists:** the motors hum and do not turn at small commands. This procedure measures the
command at which each wheel *does* start (its breakaway), on the stand. That measured number is
the evidence [proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md) needs.

---

## What you need

- **The Pi** (Ubuntu 24.04, ROS 2 Jazzy, WiringPi, the motor plugin built, the repo checked out and
  built in `~/ROCKO-env`). Three terminals on it (SSH three times).
- **The robot with both drive wheels off the ground** (on blocks, or held so the wheels spin free).
  It must not be able to drive off the bench.
- **A way to read battery voltage** (multimeter or the robot's own meter).
- **If the robot starts at boot** (`rocko.service`, [guide](run-on-the-pi.md#the-boot-service-the-supervisor)),
  stop it first: `sudo systemctl stop rocko.service`, and check with `pi/install_boot_service.sh robot status`
  (`running now: inactive`). `pi/launch.sh` refuses to start while it runs.
- **Your laptop** is not needed for this guide. The launch below uses `teleop:=false`, which leaves out the
  USB controller nodes: `TankTeleop.py` would otherwise publish its own commands (zero, 50 times a second)
  and they would fight the test commands below.

**Every terminal on the Pi runs as root, with the same `ROS_DOMAIN_ID`.** The motor plugin needs root
(WiringPi), and a terminal that is not root cannot see the launch at all. The scripts in `pi/` do both,
and set up ROS and the workspace; there is nothing to `source` by hand. Why, and what each script
does: [Run ROCKO on the Pi](run-on-the-pi.md). In every terminal, first:

```bash
cd ~/ROCKO-env
```

## How a command becomes motor power

This is the whole chain in tank mode. There is no PID in it, and no gyro.

```text
cmd_vel (m/s)
  -> diff_drive_controller: wheel speed = v / 0.127 m radius          [rad/s]
  -> Motor12VoltQuadEncoder::write (Motor12VoltQuadEncoder.cpp:231):
       duty % = wheel speed / MAX_RAD_PER_SEC x 100 + 2   (the 2 is kStictionDutyPercent)
       sent to software PWM as a whole percent (so 2.9 becomes 2)
```

`MAX_RAD_PER_SEC` is 119.4 rad/s (`Motor12VoltQuadEncoder.hpp:54`; the source line is
`1150 / 60 * ...`, and `1150 / 60` is integer division, so it is 19 and not 19.17). So for a forward
command `v` in m/s on a straight line:

```text
duty % = 6.5957 x v + 2     (truncated; 6.6 is close but crosses a whole percent at 5 and 10 m/s)
```

| `linear.x` | duty | what sends this |
| --- | --- | --- |
| 0.1 m/s | 2 % | |
| 0.5 m/s | 5 % | |
| 1.0 m/s | 8 % | |
| 2.0 m/s | 15 % | |
| 3.0 m/s | 21 % | right wheel's breakaway, 2026-11-19 |
| 4.0 m/s | 28 % | left wheel's breakaway, 2026-11-19 |
| 5.0 m/s | 34 % | |
| 10 m/s | 67 % | |

A software-PWM pulse repeats every 10 ms (100 steps of 100 microseconds: WiringPi's documented
step, not measured here), so a low duty is a short
pulse 100 times a second: that is the hum. A 12 V geared motor needs more average voltage than that
to overcome static friction ("stiction"); how much more is what this guide measures.

## Procedure

Write the battery voltage, the date and `git rev-parse --short HEAD` (and whether `git status` is
clean) at the top of your notes. [Recording a result](reproducing-results.md) lists the other fields.

### 1. Launch, and check it is healthy

Terminal A, on the Pi (it asks for your `sudo` password):

```bash
pi/launch.sh tank teleop:=false
```

Terminal B, on the Pi. `pi/rocko_shell.sh` opens a root shell with the environment; the commands that
follow in this guide are typed inside it:

```bash
pi/rocko_shell.sh
```

```bash
ros2 control list_controllers
```

Success: `joint_state_broadcaster` and `diffbot_base_controller` are both `active`, and terminal A
shows no `Could not start the interrupt on encoder pin` line. Since 2026-11-24 each motor plugin counts
its own encoder ([how](../architecture/encoders-in-cpp.md)); if its interrupts cannot start, the plugin
fails to configure and the controllers never become `active`. A wiring fault the plugin cannot see (an
unplugged encoder) shows instead as a wheel whose `position` never changes in
`ros2 topic echo /joint_states` while you turn it by hand: check that before going on, because every
wheel speed below depends on it.

### 2. Check the encoders see the wheels

In terminal B:

```bash
ros2 topic echo /joint_states --field velocity
```

Turn each wheel by hand. Success: the printed velocity for that wheel leaves zero while you turn and
returns to zero when you stop. If a wheel does not move the numbers, its encoder is not wired or read
correctly; later results for that wheel only tell you what the motor did, not how fast it went.

Leave this command running in terminal B for the rest of the guide.

### 3. Find the breakaway command

Terminal C (a third SSH session; run `pi/rocko_shell.sh` in it, as in terminal B). Each command publishes one level for 5 seconds
and stops by itself. Start at the bottom, and watch the wheels and the numbers in terminal B. Go up
one level at a time; **stop going up at the first level where both wheels turn steadily.**

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0}}}"
```

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 2.0}}}"
```

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 3.0}}}"
```

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 4.0}}}"
```

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 5.0}}}"
```

Rest 5 seconds between levels. Write down, for each wheel separately: the lowest `x` at which it
**started** turning, the lowest `x` at which it turned **steadily** (not twitching), and the encoder
velocity it settled at. The duty is `6.5957 x + 2` (truncated), or read it from the table.

The first time `ros2 topic pub` sends a message with a zero timestamp, the controller may print a
one-time warning that it is substituting the current time. That is expected. (Not checked on this robot.)

### 4. Direction and the other direction

Use the lowest level at which both wheels turned steadily. Send it forward, then backward (`x: -1.0`
and so on, same form). Success: both wheels turn the same way as each other when seen from the same
side of the robot, and the encoder velocities have the same sign for forward. If one wheel runs
backward, the `invert` parameter in `rocko_tank_mode.ros2_control.xacro` for that motor is wrong;
write it down and do not change it during this test. (Whether the encoder sign itself is right is a
separate question, [proposal 0006](../proposals/0006-velocity-loop-sign-check.md).)

### 5. Turning

```bash
timeout 5 ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 4.0}}}"
```

`angular.z` is in rad/s and the wheels are 0.32 m apart, so each wheel gets 4.0 x 0.16 = 0.64 m/s in
opposite directions. Write down whether the wheels turn opposite to each other. This is the number to
compare with the controller's turn. Tank mode does not launch `DiffDriveController.py`, so
`diff_drive_controller` reads `angular.z` as rad/s, which is what the arithmetic above uses.

### 6. Stop

Ctrl-C in terminal A. On shutdown the plugin writes 0 % and stops the PWM thread
(`Motor12VoltQuadEncoder.cpp:172-173`), so the wheels should stop. (While the launch is still running,
the controller's own 1 s timeout, `rocko_tank_mode_controllers.yaml:33`, stops them if commands stop.)
If a wheel keeps turning after the launch has exited, switch the motor supply off: that would be a
real finding (a PWM thread left running).

## What to bring back

Copy this into your notes, one row per wheel, and attach it to proposal 0013.

```text
Date / commit / tree clean?:
Battery volts at start / end:
Surface / supports:
                         left wheel        right wheel
Started turning at x =
Steady at x =
Encoder velocity there =
Direction OK?
Turn test (z = 4.0):
Anything else (heat, smell, noise, a wheel that stuck):
```

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| Nothing at any level up to 5.0 | Battery low, motor supply off, a driver not enabled, or a wiring fault | Measure the battery under no load; check the motor supply switch; swap to a known-good motor channel. This is a hardware finding, not a software one |
| The wheels turn here but barely move under the controller at small stick | Small stick is below your breakaway | Expected; that is proposal 0013 |
| The wheels jerk at each step instead of turning smoothly | Duty is just above breakaway; the wheel turns, sticks, turns | Go one level higher; note both levels |
| Two sources of commands: the wheels twitch between values | The launch was started without `teleop:=false`, so `TankTeleop.py` is publishing too | Stop the launch and start it again with `teleop:=false` |
| `Could not start the interrupt on encoder pin ...`, or a wheel's `position` never changes in `ros2 topic echo /joint_states` | The motor plugin counts its encoder itself since 2026-11-24 ([how](../architecture/encoders-in-cpp.md)): the interrupt could not start, or the pin numbers or wiring are wrong | Check the launch log for the motor plugin's error; compare `encoder_pin_a`/`encoder_pin_b` in the xacro with `gpio readall` |
| `ros2 control list_controllers` prints `waiting for service /controller_manager/list_controllers` and never answers | This terminal is not root, or uses another `ROS_DOMAIN_ID`, so it cannot see the launch; or the launch in terminal A has stopped | Open the terminal with `pi/rocko_shell.sh`; check terminal A is still running |

## What this does not tell you

It says nothing about balancing: tank mode has no balance loop. It does not check how fast the wheels
really turn above breakaway (the encoder numbers do, if the encoders are right, and that is checked
only by hand-turning in step 2). It is a bench test with no load; the duty needed to move the robot
on the floor is higher.
