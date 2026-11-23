# Drive tank mode with a USB game controller on the Pi (8BitDo)

A game controller plugged into the Raspberry Pi's USB drives ROCKO in **tank mode** (balancing off, plain
wheel driving). Nothing runs on your laptop: the Pi reads the controller itself. This is the only way to drive
the robot on this branch since 2026-11-22 (keyboard and laptop driving were removed).

**What has been checked (2026-11-22):** **on the robot, on blocks, the owner drove it** (owner-run, battery voltage
not recorded): forward, reverse, turning, LB release and the unplug/re-arm all worked, and a capture (not kept) matched
every stick direction to its command ([journal](../journal/2026-11-22-tank-reversal-lag.md)).
Before that, the stick arithmetic, the left-bumper rule and the stop on silence, with
fake controller messages (`tools/tank_teleop_test.py`); that tank mode starts on the mock with both new nodes up
and no controller attached (Tier 2); that the dashboard shows a controller and the enable state (a fake one, in
the container). **Not checked:** a real 8BitDo, the signs of its sticks, and the robot. The evidence is in the
[journal](../journal/2026-11-22-usb-controller-tank-mode.md).

## How you drive it: split arcade, hold LB

| Control | What it does |
| --- | --- |
| **LB** (left bumper, L1), held | Drive enable. Let go and the command goes to zero at once (the wheels then slow down: see *What stops the robot*) |
| **Left stick**, up and down | Forward and back |
| **Right stick**, left and right | Turn left and right |
| Everything else, RB included | Nothing |

After the robot starts, and after the controller is unplugged and plugged back in, **let go of LB once** before
it will drive. That stops a bumper that was held (or that the driver still reports as held) from driving the robot
off by itself. The dashboard says so when it happens.

## How it works

```text
8BitDo --USB--> joy's game_controller_node --/joy, 20 Hz--> TankTeleop.py --cmd_vel, 50 Hz--> diff_drive_controller
                                                                 \--tank_teleop/enabled--> dashboard
```

- **`game_controller_node`** (ROS package `joy`) reads the controller through SDL's GameController layer, which
  reports every supported pad in one standard layout. While a controller is connected it republishes its state
  20 times a second; when it is unplugged it goes silent.
- **`TankTeleop.py`** (tank mode only) turns that into a command: deadzone and expo on each stick, mixing so you can
  still turn at full speed, and **100 % motor duty at full stick** (15 m/s, 93.75 rad/s for a turn alone; the
  arithmetic is in the file, and raising tank mode's top speed is
  [proposal 0014](../proposals/0014-tank-mode-full-speed-for-shaped-sticks.md), waiting for the owner's
  confirmation). It publishes zero unless LB is held, and zero when nothing has arrived on `/joy` for 0.25 s.
- **What both modes share** (the controller's index layout, the shaping and mixing, the hold-to-drive rule) is in
  [`rocko_env/teleop/gamepad.py`](../../rocko_env/rocko_env/teleop/gamepad.py). Balancing mode does not start
  either node. It has no controller input yet (see *What this does not cover*).

**What each stick position gives, driving straight** (defaults: deadzone 0.10, expo 0.6; arithmetic from the code,
not a measurement):

| Left stick pushed [%] | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Motor duty [%] | 0 | 6 | 11 | 17 | 24 | 34 | 45 | 60 | 78 | 100 |
| ... with `min_output` 0.2 | 0 | 25 | 29 | 34 | 40 | 47 | 56 | 68 | 83 | 100 |

**The motors only hum below about 21 % (right) and 28 % (left) duty** (bench, 2026-11-19,
[proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md)). With the defaults that is the first half of the
stick. `min_output` 0.2 starts the first movement past the deadzone at about 22 % duty instead.

## Set up, once

**The controller.** 8BitDo pads have several modes. Use **X-input** (Xbox) mode: Linux's built-in `xpad` driver
then treats it as an Xbox controller, and SDL maps it by that name. How to select it depends on the model (a
switch marked X on the back of the Pro 2, a button held while plugging in on others); see its manual. The
D-input, Switch and macOS modes may also work, but with an unknown mapping.

**The Pi** (Ubuntu 24.04 + Jazzy), in a terminal, as your own user (not root; nothing to `source` first:
`build_ros.sh` sets up ROS itself, as [Run ROCKO on the Pi](run-on-the-pi.md) explains). The checkout must be on
the `ai-forward` branch (`git branch` shows it with a `*`):

```bash
sudo apt install ros-jazzy-joy
```

```bash
cd ~/ROCKO-env && git pull && ./build_ros.sh
```

Success: the build ends with `Done!`; a failed build prints `ERROR: colcon build failed` instead. Do this before the first
`pi/launch.sh tank` after pulling: tank mode now starts `TankTeleop.py`, which an older build does not have.

## First: check what the Pi sees (no driving)

Plug the controller into the Pi. Then check that Linux sees it:

```bash
ls /dev/input/by-id/ | grep -i joystick
```

Success is one or more names ending in `-event-joystick`. Then start the reader on its own, as root with the robot's
environment ([Run ROCKO on the Pi](run-on-the-pi.md)):

```bash
pi/rocko_run.sh ros2 run joy game_controller_node
```

Success: a line like `Controller Found: 8BitDo Pro 2 Wired Controller, ...` (the name varies). In a second terminal
on the Pi:

```bash
pi/rocko_run.sh ros2 topic echo /joy
```

Move the sticks and press LB, and check against the layout in `gamepad.py`:

1. Left stick **up**: `axes[1]` goes to about **+1.0**. Right stick **left**: `axes[2]` goes to about **+1.0**
   (right is -1.0).
2. LB pressed: `buttons[9]` is 1.
3. Hands off: every stick axis rests within 0.10 of zero (otherwise raise `deadzone`).

If a stick reads the other way round, or LB is a different index, **stop and report it**: the mapping comes from SDL,
and a wrong one would drive the wrong way. Ctrl-C both when done.

## Drive

**The first time, wheels off the ground** (the robot on a stand, as in the [bench test](bench-test-tank-mode.md)), with
the controller plugged in and low caps. On the Pi:

```bash
pi/launch.sh tank max_speed:=0.4 max_turn:=1.0
```

`max_speed` caps every wheel at that fraction of full speed and `max_turn` weakens the turn stick (both default 1.0).
**The two multiply for turning:** a full turn alone spins each wheel at `max_speed x max_turn` of full speed. Keep
that product above about 0.3, or turning stays inside the motors' breakaway band and only hums (it did on the first
stand test with 0.4 x 0.3: [journal](../journal/2026-11-22-tank-reversal-lag.md)).
Success looks like this in the launch output:

```text
[game_controller_node-N] ... Controller Found: ...
[TankTeleop.py-N] ... tank teleop: hold LB to drive; settings {...}
[TankTeleop.py-N] ... controller heard on /joy
[TankTeleop.py-N] ... DRIVE ENABLED (LB held)
```

Open the dashboard (`http://<robot-ip>:8080`) on a laptop or phone: the **Controller** card shows the pad live and
says ENABLED, Disabled, or No controller.

On the stand, check in this order, and **record the battery voltage and the commit** as
[reproducing results](reproducing-results.md) asks:

1. Nothing turns until LB is held. RB does nothing.
2. Left stick up: both wheels turn the way that drives the robot forward. Right stick right: the left wheel
   forward and the right wheel back (a right turn).
3. Let go of LB with the stick held: both wheels slow to a stop.
4. Hold LB and the stick, then **unplug the controller**. The launch prints `controller gone, stopping` and the wheels
   stop. Plug it back in while still holding LB: the robot must **not** drive until you let go of LB and hold it again.
5. Raise the caps in steps. Only then put the robot on the floor, with space around it.

## What stops the robot

| Event | What happens |
| --- | --- |
| LB released | A zero command within 20 ms. The wheels then slow at tank mode's deceleration limit (`linear.x.max_deceleration`, [proposal 0015](../proposals/0015-tank-mode-faster-reversal.md)); a turn stops almost at once |
| Controller unplugged, or `game_controller_node` stops | Nothing arrives on `/joy`; after 0.25 s `TankTeleop.py` commands zero and needs LB released before driving again |
| `TankTeleop.py` stops | `diff_drive_controller` zeroes the wheels after its `cmd_vel_timeout` of 1.0 s |
| The launch is stopped (Ctrl-C on the Pi) | Everything stops |

Tank mode limits turning acceleration to 1000 rad/s², so a full turn reaches its speed in about a tenth of a second
(proposal 0014). **Reversing:** `diff_drive_controller` ramps forward/back changes. The ramp was made three times steeper on 2026-11-22
(the owner found full forward to full reverse slow); the numbers, before and after, are in
[proposal 0015](../proposals/0015-tank-mode-faster-reversal.md). A short dead spot remains while the command crosses the
motors' breakaway band (proposal 0013).

## What goes wrong

- **No `Controller Found` line.** The pad is in the wrong mode, or the cable is charge-only. Check
  `ls /dev/input/by-id/` again; try X-input mode and another cable. Plugging it in while the launch runs is fine:
  the joy node finds it without a restart.
- **`package 'joy' not found`** in the launch output. `sudo apt install ros-jazzy-joy`, then stop the launch
  (Ctrl-C) and start it again.
- **The sticks are swapped or reversed.** The pad is probably not in X-input mode, so SDL is guessing. Change the mode
  and check again with `ros2 topic echo /joy`. Do not drive until the first-check list passes.
- **`No module named 'rocko_env.teleop'`** in the launch output. The build predates this change's new folder:
  rebuild from clean on the Pi, in `~/ROCKO-env`: `rm -rf build install log && ./build_ros.sh`.
- **The dashboard says "LB reads held but driving is not allowed yet".** The re-arm rule: release LB once, then hold it.
- **The dashboard says "Teleop not running".** Not tank mode, or `TankTeleop.py` died: read the launch output.
- **The wheels hum and do not turn at small stick.** Expected below 21 to 28 % duty: see the table and `min_output`.

## What this does not cover

**Balancing mode.** It keeps its own nodes (`Joystick.py`, `DiffDriveController.py`, with the balancing speed limits).
`Joystick.py` still listens for a laptop over TCP, but no sender for it is left in the repo, so balancing mode
cannot be driven at present. Giving it the USB controller is a separate, later change: [next-steps.md](../next-steps.md).

## Where the numbers live

| Number | Where |
| --- | --- |
| Deadzone, expo, minimum output, caps (ROS parameters of `tank_teleop`) | `controllers/tank_teleop/TankTeleop.py`, `DEFAULTS`; `max_speed:=` and `max_turn:=` on the launch command line |
| Full speed and full turn in tank mode, the deadman button, the publish rate | `TankTeleop.py`, the constants at the top |
| Which `/joy` index is which stick or button; the joy node's rate and the 0.25 s timeout | `rocko_env/teleop/gamepad.py` |
| The motor's speed-to-duty formula | `Motor12VoltQuadEncoder::write` |
