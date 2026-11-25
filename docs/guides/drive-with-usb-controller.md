# Drive ROCKO with a USB game controller on the Pi (8BitDo)

A game controller plugged into the Raspberry Pi's USB drives ROCKO. Nothing runs on your laptop: the Pi reads the
controller itself. Since 2026-11-23 a **supervisor** program owns what the robot is doing: the robot boots into
**tank mode** (balancing off, plain wheel driving), **disabled**, and you enable it and switch modes with
controller button combinations or from the [dashboard](use-the-dashboard.md). How that works and why:
[the robot supervisor](../architecture/robot-supervisor.md).

**What has been checked.** On 2026-11-22, with the old hold-LB rule, **on the robot, on blocks, the owner drove it**
(owner-run, battery voltage not recorded): forward, reverse, turning, the LB release and the unplug/re-arm all worked,
and a capture (not kept) matched every stick direction to its command
([journal](../journal/2026-11-22-tank-reversal-lag.md)). **The 2026-11-23 change has not run on the Pi or the robot.**
It was checked with fake controller messages and the mock only; what ran, and what each check can and cannot show, is in
[the journal](../journal/2026-11-23-robot-supervisor.md) and the
[verification ladder](../architecture/verification-ladder.md). **Not checked:** the LB+RB and LT+RT+RB combos on a
real 8BitDo (the trigger "pressed" reading is inferred from the joy node's code), the signs of its sticks, and
anything on the robot.

## How you drive it: enable with LB + RB, then split arcade

| Control | What it does |
| --- | --- |
| **LB + RB** pressed together (the triggers not pressed) | **Toggle enable.** Disabled becomes enabled, enabled becomes disabled. Same in both modes |
| **LT + RT + RB** pressed together (LB not pressed) | **Toggle tank / balance.** Only while the robot is **disabled**; ignored otherwise |
| **Left stick**, up and down | Forward and back (while enabled) |
| **Right stick**, left and right | Turn left and right (while enabled) |
| Everything else | Nothing |

- A combo fires **once, the moment its last button goes down**. To fire it again, release one of its buttons first.
  A combo already held when the controller is first heard (start-up, or after a re-plug) does not fire: release it first.
- **After you enable, both sticks must return to the centre once before the robot moves.** A stick that was held over
  when you pressed the combo does not make the robot lurch. The dashboard says so.
- **Letting go of a button no longer stops the robot.** Until 2026-11-23, tank mode drove only while LB was held. That
  deadman was replaced by the enable toggle (the owner's decision: [ADR 0006](../decisions/0006-robot-supervisor.md)).
  **To stop: press LB + RB again, or press Disable on the dashboard.** The other things that stop it are in
  *What stops the robot* below.

## How it works

```text
8BitDo --USB--> joy's game_controller_node --/joy, 20 Hz--> TankTeleop.py ----cmd_vel, 50 Hz--> diff_drive_controller
                        |                                   BalanceTeleop.py --cmd_vel--> DiffDriveController.py (balance)
                        +--/joy--> Supervisor.py  (LB+RB, LT+RT+RB)  --/rocko/state--> teleop nodes, dashboard
```

- **`game_controller_node`** (ROS package `joy`) reads the controller through SDL's GameController layer, which
  reports every supported pad in one standard layout. While a controller is connected it republishes its state
  20 times a second; when it is unplugged it goes silent. It runs in every mode.
- **`Supervisor.py`** watches `/joy` for the two combos, decides whether they are allowed, starts and stops the
  motor stack, and publishes the result on `/rocko/state`. The teleop nodes publish zero until that state says the
  robot is enabled in their mode.
- **`TankTeleop.py`** (tank mode) turns the sticks into a command: deadzone and expo on each stick, mixing so you can
  still turn at full speed, and **100 % motor duty at full stick** (15 m/s, 93.75 rad/s for a turn alone; the
  arithmetic is in the file, and raising tank mode's top speed is
  [proposal 0014](../proposals/0014-tank-mode-full-speed-for-shaped-sticks.md), waiting for the owner's
  confirmation). It publishes zero unless the robot is enabled and the sticks have been centred since, and zero when
  nothing has arrived on `/joy` for 0.25 s.
- **`BalanceTeleop.py`** (balancing mode) is the same idea with its own, lower speeds: see *Balancing mode* below.
- **What both modes share** (the controller's index layout, the shaping and mixing, the combo and centre rules) is in
  [`rocko_env/teleop/gamepad.py`](../../rocko_env/rocko_env/teleop/gamepad.py).

**What each stick position gives, driving straight in tank mode** (defaults: deadzone 0.10, expo 0.6; arithmetic from
the code, not a measurement):

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
launch after pulling: the supervisor and `BalanceTeleop.py` are new, and an older build does not have them (nor the
new `rocko_interfaces` messages).

## First: check what the Pi sees (no driving)

Plug the controller into the Pi. Then check that Linux sees it:

```bash
ls /dev/input/by-id/ | grep -i joystick
```

Success is one or more names ending in `-event-joystick`. If `rocko.service` is running (see *Start at boot* below) it
has already started the reader, so skip the next command and go to the `ros2 topic echo`. Otherwise start the reader on
its own, as root with the robot's environment ([Run ROCKO on the Pi](run-on-the-pi.md)):

```bash
pi/rocko_run.sh ros2 run joy game_controller_node
```

Success: a line like `Controller Found: 8BitDo Pro 2 Wired Controller, ...` (the name varies). In a second terminal
on the Pi:

```bash
pi/rocko_run.sh ros2 topic echo /joy
```

Move the sticks and press the buttons, and check against the layout in `gamepad.py`:

1. Left stick **up**: `axes[1]` goes to about **+1.0**. Right stick **left**: `axes[2]` goes to about **+1.0**
   (right is -1.0).
2. LB pressed: `buttons[9]` is 1. RB pressed: `buttons[10]` is 1.
3. LT and RT pulled right in: `axes[4]` and `axes[5]` each go well above 0.5 (the supervisor reads a trigger as
   pressed above 0.5; that reading is from the joy node's code, not seen on an 8BitDo).
4. Hands off: every stick axis rests within 0.10 of zero (otherwise raise `deadzone`).

If a stick reads the other way round, or a button or trigger is a different index, **stop and report it**: the mapping
comes from SDL, and a wrong one would drive the wrong way or fire the wrong combo. Ctrl-C both when done.

## Drive

**The first time, wheels off the ground** (the robot on a stand, as in the [bench test](bench-test-tank-mode.md)), with
the controller plugged in. The supervisor starts tank mode with the default caps (full speed); **the launch arguments
`max_speed:=` and `max_turn:=` do not reach tank mode through `rocko_robot.launch.py`**, so on the stand push the
sticks gently. (Caps are a gap: see *What this does not cover*.) On the Pi, either let the boot service start it
(next section) or run it by hand:

```bash
pi/launch.sh robot
```

Success looks like this in the launch output (the tank teleop line says it follows the supervisor):

```text
[game_controller_node-N] ... Controller Found: ...
[TankTeleop.py-N] ... tank teleop: drive when /rocko/state says enabled in tank mode; settings {...}
[TankTeleop.py-N] ... controller heard on /joy
```

Open the dashboard (`http://<robot-ip>:8080`) on a laptop or phone: the **state card** at the top says tank,
DISABLED, phase ready; the **Controller** card shows the pad live. See [Watch the robot in a browser](use-the-dashboard.md).

On the stand, check in this order, and **record the battery voltage and the commit** as
[reproducing results](reproducing-results.md) asks:

1. Disabled: push both sticks; nothing turns. RB alone and LB alone do nothing.
2. Press **LB + RB**. The state card says ENABLED. Hold the left stick over while you press it: nothing turns until
   you let both sticks return to the centre (the dashboard says the sticks must centre).
3. Left stick up: both wheels turn the way that drives the robot forward. Right stick right: the left wheel
   forward and the right wheel back (a right turn).
4. Release the sticks: the wheels slow to a stop. Press **LB + RB** with the stick held: the wheels stop (DISABLED).
5. Enable, hold a stick, then **unplug the controller**. The wheels stop (the supervisor
   disables when the controller goes silent in tank mode), and the state card says DISABLED, last change
   `supervisor`. Plug it back in: it stays disabled until you press LB + RB again.
6. Press **LB + RB** while the robot is moving and the dashboard's **Disable** button too, once each, to see both stop it.
7. Only then put the robot on the floor, with space around it.

## Start at boot (the supervisor service)

To have the robot come up whenever it is switched on, with nothing to type, install the boot service once. On the
**Pi**, as your own user (not root), from `~/ROCKO-env`:

```bash
pi/install_boot_service.sh robot
```

Success: `Installed /etc/systemd/system/rocko.service; enabled at boot; started.` It starts **now** as well, so do
this with the robot on blocks or with space around it. If the older `rocko-tank.service` or `rocko-dashboard.service`
is installed, the script removes them first (it prints which); **the new service replaces both**, because two
launches would fight over the motors and two dashboards over the port. Check it:

```bash
pi/install_boot_service.sh robot status
```

Success: `enabled at boot: enabled`, `running now: active`, `restarts: 0`. After every boot the robot is in tank mode,
**disabled**, within about half a minute. Plug in the controller (any time) and press **LB + RB** to drive. The full
procedure (migration, removing, the launch guard) is in [Run ROCKO on the Pi](run-on-the-pi.md#the-boot-service-the-supervisor).

- It runs exactly `pi/launch.sh robot`. The robot never moves at boot: nothing is enabled until a person presses LB + RB
  or the dashboard's Enable. (A boot with LB + RB held does not fire the combo either: release it first.)
- If the launch itself fails, the service tries again at most 3 times in 2 minutes, then stays stopped (status shows
  it). A node that crashes inside a running launch does not restart the launch: the supervisor notices a missing node,
  disables and reports a fault on the dashboard; read the log (`sudo journalctl -u rocko.service`).
- **While it runs, `pi/launch.sh` refuses to start any other launch** (a second one would fight over the motors). To
  run a bench test by hand: `sudo systemctl stop rocko.service` (it comes back at the next boot), or
  `pi/install_boot_service.sh robot remove` to stop starting it at boot.

## What stops the robot

| Event | What happens |
| --- | --- |
| **LB + RB** pressed, or **Disable** on the dashboard | The controllers are deactivated and the wheels are commanded to zero. In tank mode the wheels then slow at the drive acceleration limit (`linear.x.max_deceleration`, [proposal 0015](../proposals/0015-tank-mode-faster-reversal.md)); a turn stops almost at once. In balance mode disabling means **the robot falls**: it has nothing holding it up |
| Controller unplugged, or `game_controller_node` stops (**tank mode**) | Nothing arrives on `/joy`. The supervisor disables after 0.25 s and the teleop node commands zero. Re-enable with LB + RB |
| Controller unplugged (**balance mode**) | The supervisor does **not** disable (that would drop the robot); the teleop node commands zero velocity and the robot holds still, upright. See *Balancing mode* |
| A node or controller of the running stack goes missing | The supervisor reports `fault`. In tank mode it also disables. In balance mode it stays enabled unless a cascade controller was lost: the message on the dashboard says which |
| `controller_manager` stops answering | Counted as unknown, not missing. After 3 s it is a `fault`. In tank mode it also disables and, if it is still silent 5 s later, the whole stack is killed. In balance mode the robot stays enabled and the stack is not killed (if it was already disabled, it is killed as in tank mode) |
| A disable the supervisor cannot confirm | Every disable is checked through `list_controllers` and retried. If it is not confirmed within 6 s the whole stack is killed, which stops the wheels |
| `TankTeleop.py` stops | `diff_drive_controller` zeroes the wheels after its `cmd_vel_timeout` of 1.0 s |
| The launch is stopped (Ctrl-C on the Pi, or `sudo systemctl stop rocko.service` for the boot service) | Everything stops. When a stack's launch dies, its whole process group is killed |
| Closing the dashboard page | **Nothing.** The dashboard is not a deadman |

Tank mode limits turning acceleration to 1000 rad/s², so a full turn reaches its speed in about a tenth of a second
(proposal 0014). **Reversing:** `diff_drive_controller` ramps forward/back changes. On 2026-11-22 the owner found full forward to full
reverse slow and the ramp was raised to match turning; the numbers, before and after, are in
[proposal 0015](../proposals/0015-tank-mode-faster-reversal.md). A short dead spot remains while the command crosses the
motors' breakaway band (proposal 0013).

**What is weaker than before.** A held button used to stop the robot when you let go. A toggle cannot. If you lose the
controller you rely on the automatic disable in the table; if you lose sight of the robot, press LB + RB. The dashboard
has **no login**: anyone on the network can enable the robot ([the dashboard guide](use-the-dashboard.md#what-the-page-can-and-cannot-do)).

## Balancing mode

**Balance mode has never balanced on the robot.** What changed on 2026-11-23 is only that it is reachable and has a
driving input; it is **untested**. This guide does not give a balancing procedure. The order of work before a first
tethered test is the queue in [next-steps.md](../next-steps.md) (its HANDOFF box at the top).

What exists, so you know what a press does:

- Switch to it with **LT + RT + RB** (or the dashboard's **Balance**) while the robot is disabled. The supervisor
  stops tank mode's stack and starts balancing's. This takes several seconds, with no motor stack in between.
- **Enabling in balance mode needs the robot near upright and held still** (`|pitch|` within the tipover cutoff's re-arm
  band, read from the controller, and the pitch rate within `ENABLE_MAX_PITCH_RATE_DEG_S` in
  `supervisor/state_machine.py`); otherwise the enable is refused and the message says which ("hold the robot still").
- `BalanceTeleop.py` reads the same pad: **left stick Y** is forward speed and **right stick X** is turn, with the
  same deadzone, expo, enable and centre rules. Its defaults are `max_speed` 0.05 (0.5 m/s) and `max_turn` 0.2, **untested
  starting points**, not tuned values. They are launch arguments of `rocko.launch.py` (`max_speed:=`, `max_turn:=`,
  `teleop:=`).
- Disabling in balance mode means the robot **falls**. Never disable it unless it is held.

**Driving by hand without the supervisor.** `rocko.launch.py` run by hand (`supervised:=false`) no longer starts
`Joystick.py` (the old laptop TCP path). It starts `game_controller_node` and `BalanceTeleop.py` instead, but with no
supervisor publishing `/rocko/state` the teleop node never enables. To drive, run `pi/launch.sh robot` and switch to
balance mode. The simulation launch (`rocko_sim.launch.py`) still uses `Joystick.py`.

## What goes wrong

- **No `Controller Found` line.** The pad is in the wrong mode, or the cable is charge-only. Check
  `ls /dev/input/by-id/` again; try X-input mode and another cable. Plugging it in while the launch runs is fine:
  the joy node finds it without a restart.
- **`package 'joy' not found`** in the launch output. `sudo apt install ros-jazzy-joy`, then stop the launch
  (Ctrl-C) and start it again.
- **The sticks are swapped or reversed.** The pad is probably not in X-input mode, so SDL is guessing. Change the mode
  and check again with `ros2 topic echo /joy`. Do not drive until the first-check list passes.
- **`No module named 'rocko_env.teleop'` or `rocko_interfaces.msg`** in the launch output. The build predates the new
  folders or messages: rebuild from clean on the Pi, in `~/ROCKO-env`: `rm -rf build install log && ./build_ros.sh`.
- **Enabled, but the wheels do not move.** The sticks have not been centred since you enabled. Let both go to the centre.
- **LB + RB does nothing.** The robot is not `ready` (the state card's phase says why), or the combo was already held
  when the controller was plugged in: release both and press again. Pressing a trigger with them (LT or RT) makes it a
  different combo.
- **LT + RT + RB does nothing.** The robot must be disabled and ready. The dashboard's message says which refused it.
- **The state card says NO SUPERVISOR.** This launch was started by hand (`pi/launch.sh tank` or `balance`), or the
  supervisor died. Controls will not enable the robot; use `pi/launch.sh robot`.
- **The dashboard says "Teleop not running".** The teleop node of the current mode is silent: read the launch output.
- **The wheels hum and do not turn at small stick.** Expected below 21 to 28 % duty: see the table and `min_output`.

## What this does not cover

- **Speed caps for tank mode.** `max_speed:=` and `max_turn:=` are launch arguments of `rocko_tank_mode.launch.py`, and
  `rocko_robot.launch.py` does not pass them on. A hand launch of tank mode accepts them, but cannot be driven from
  the controller (no supervisor, so the teleop node never enables).
- **Balancing itself** (above), and anything about it on the robot.

## Where the numbers live

| Number | Where |
| --- | --- |
| Deadzone, expo, minimum output, caps (ROS parameters of `tank_teleop` and `balance_teleop`) | `controllers/tank_teleop/TankTeleop.py` and `controllers/balance_teleop/BalanceTeleop.py`, `DEFAULTS`; `max_speed:=` and `max_turn:=` on a launch command line |
| Full speed and full turn in tank mode, the publish rate | `TankTeleop.py`, the constants at the top |
| Which `/joy` index is which stick or button; the combos; the joy node's rate and the 0.25 s timeout | `rocko_env/teleop/gamepad.py` |
| Enable rules, the pitch band, timeouts of the supervisor | [robot-supervisor.md](../architecture/robot-supervisor.md); `rocko_env/supervisor/state_machine.py` and `modes.py` |
| The motor's speed-to-duty formula | `Motor12VoltQuadEncoder::write` |
