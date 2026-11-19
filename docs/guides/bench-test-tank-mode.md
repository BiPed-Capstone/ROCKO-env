# Bench-test tank mode: do the wheels turn, and at what command?

**Who this is for:** the person at the robot the first time `rocko_tank_mode.launch.py` runs, or
anyone whose wheels hum and do not turn.

**What has been checked:** nothing in this guide has been run. The numbers come from reading the
code (they are worked out below so you can check them), not from a robot. Every step that touches the
robot is yours: an agent on this branch does not launch anything on the Pi
([CLAUDE.md](../../CLAUDE.md) section 3).

**Why it exists:** the keyboard script drives tank mode with a command so small that the motors get
about 2 to 5 % of full power, which is not enough to start the wheels turning. This procedure
confirms that, and measures the command at which each wheel *does* start. That measured number is
the evidence [proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md) needs.

---

## What you need

- **The Pi** (Ubuntu 24.04, ROS 2 Jazzy, WiringPi, the motor plugin built, the repo checked out and
  built in `~/ROCKO-env`). Three terminals on it (SSH three times).
- **The robot with both drive wheels off the ground** (on blocks, or held so the wheels spin free).
  It must not be able to drive off the bench.
- **A way to read battery voltage** (multimeter or the robot's own meter).
- **Your laptop** is not needed for this guide. Leave `keyboard_teleop.py` and
  `computer_joystick.py` **stopped**: while a sender is connected, `Joystick.py` publishes its own
  commands and they fight the test commands below.

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
| 0.1 m/s | 2 % | keyboard, one `w` press |
| 0.5 m/s | 5 % | keyboard, five presses (the most it can do) |
| 1.0 m/s | 8 % | |
| 2.0 m/s | 15 % | |
| 3.0 m/s | 21 % | |
| 4.0 m/s | 28 % | |
| 5.0 m/s | 34 % | |
| 10 m/s | 67 % | a full forward push on the gamepad |

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
pi/launch.sh tank
```

Terminal B, on the Pi. `pi/rocko_shell.sh` opens a root shell with the environment; the commands that
follow in this guide are typed inside it:

```bash
pi/rocko_shell.sh
```

```bash
ros2 control list_controllers
```

Success: `joint_state_broadcaster` and `diffbot_base_controller` are both `active`, **and** terminal A
shows no `Failed to call service QuadEncoder` for 10 seconds. Both conditions are needed: the plugin
logs that error but still reports success (`Motor12VoltQuadEncoder.cpp:196-199`), so the controllers
stay `active` while the encoders are dead. If the error repeats, the encoder node is not answering
the motor plugin: stop here and fix that first, because every wheel speed below depends on it.

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
one-time warning that it is substituting the current time. That is expected; it is the same message
`Joystick.py` sends. (Not checked on this robot.)

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
compare with the keyboard's turn: a full `a` or `d` is `angular.z` = 0.3 (`Joystick.py:233`, `DEFAULT_PRECISE_ANGULAR_SCALE_M_S`), which is
0.048 m/s per wheel, about 2 % duty. The comment at `Joystick.py:49` says this value is "NOT rad/s"; that
holds when `DiffDriveController.py` (balance mode) reads it. Tank mode does not launch that node, so
here `diff_drive_controller` reads it as rad/s, which is what the arithmetic above uses.

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
| The wheels turn but a keyboard still seems to do nothing | The keyboard's top speed (0.5 m/s, about 5 %) is below your breakaway | Expected; that is proposal 0013 |
| The wheels jerk at each step instead of turning smoothly | Duty is just above breakaway; the wheel turns, sticks, turns | Go one level higher; note both levels |
| Two sources of commands: the wheels twitch between values | `keyboard_teleop.py` or `computer_joystick.py` is still connected | Stop it; wait a second; repeat |
| `Failed to call service QuadEncoder` | The encoder node (`QuadEncoders.py`) is not running or not answering | `ros2 node list` shows whether it is up; read its log |
| `ros2 control list_controllers` prints `waiting for service /controller_manager/list_controllers` and never answers | This terminal is not root, or uses another `ROS_DOMAIN_ID`, so it cannot see the launch; or the launch in terminal A has stopped | Open the terminal with `pi/rocko_shell.sh`; check terminal A is still running |

## What this does not tell you

It says nothing about balancing: tank mode has no balance loop. It does not check how fast the wheels
really turn above breakaway (the encoder numbers do, if the encoders are right, and that is checked
only by hand-turning in step 2). It is a bench test with no load; the duty needed to move the robot
on the floor is higher.
