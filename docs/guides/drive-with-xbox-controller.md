# Drive the robot in tank mode with an Xbox controller (macOS, Bluetooth)

An Xbox controller paired with your Mac drives ROCKO in **tank mode** (balancing off, plain wheel driving).
The program, `joystick/xbox_teleop.py`, runs on the Mac and sends the robot the same 40-byte packets the
older gamepad script sends, over TCP to `Joystick.py` on the Pi.

**What has been checked (2026-11-20):** the stick arithmetic, the mixing and the safety behaviour, with a
fake controller (`tools/xbox_teleop_test.py`); what the robot's `Joystick.py` does with the packets
(`tools/teleop_watchdog_test.py`); that both launch files still start on the mock (Tier 2); and that the
program finds no controller cleanly when none is on. **Not checked:** a real controller (it was switched off
while this was written, so no real stick value has been read), and the robot. The evidence is in the
[journal](../journal/2026-11-20-xbox-controller-split-arcade.md).

**Balancing mode is not covered.** The program is meant for tank mode. In balancing mode its full stick asks
for exactly what the old gamepad's full stick does, but nobody has driven the balancing robot with it.

## How you drive it: split arcade

| Control | What it does |
| --- | --- |
| **LB or RB** (either bumper), held | Drive enable. Let go and a zero command is sent at once (the wheels slow down; see *What stops the robot*) |
| **Left stick**, up and down | Forward and back |
| **Right stick**, left and right | Turn left and right |
| **Menu** (the small button right of the Xbox button) | Stop, then quit |
| Ctrl-C in the Terminal | Stop, then quit |
| Everything else | Nothing |

"Split arcade" means one stick per job: the left thumb sets speed, the right thumb steers. Neither stick's
other direction does anything, so pushing the left stick a little sideways does not turn the robot.

## What a stick position becomes

Three steps on the Mac, then one on the robot. Each step's number is a command-line option.

1. **Deadzone** (`--deadzone`, default 0.10). A stick at rest is rarely at exactly zero; anything under 10 % of
   its travel counts as zero. What is left is stretched back to 0..1, so the command starts from zero at the
   deadzone's edge instead of jumping.
2. **Expo** (`--expo`, default 0.6): command = 0.4 x stick + 0.6 x stick³. A full stick is still full speed;
   small stick movements give much smaller commands, which is what makes slow driving controllable. 0 is a
   straight line.
3. **Mixing.** Each wheel's speed is forward minus turn (left wheel) and forward plus turn (right wheel). At
   full forward any turn would ask the outer wheel for more than 100 %, and the turn would be lost when the
   motor clips. So when the two add up to more than full, both are scaled down together. You can always turn.
4. **On the robot**, a full command is **100 % motor duty** in tank mode: 15 m/s forward, or 93.75 rad/s for a
   turn alone (`rocko_tank_mode.launch.py`, where the arithmetic is written out). The old gamepad stopped at
   about 68 %; raising it was done at the owner's request and waits for their confirmation ([proposal 0014](../proposals/0014-tank-mode-full-speed-for-shaped-sticks.md)). A packet from this program
   carries a flag (button byte 14) that tells `Joystick.py` the stick is already shaped, so it does not apply
   its own expo curve a second time. The keyboard and the old gamepad script do not set it and are unchanged.

`--max-speed` (default 1.0) caps every wheel at that fraction of full; `--max-turn` (default 1.0) weakens the
turn stick. Start low, the turn especially: tank mode limits forward acceleration to 10 m/s² (about 1.5 s to
full speed) but turning to 1000 rad/s², so a full turn reaches full duty in about a tenth of a second.

**What each stick position gives, driving straight** (defaults; arithmetic from the code, not a measurement):

| Left stick pushed [%] | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Motor duty [%] | 0 | 6 | 11 | 17 | 24 | 34 | 45 | 60 | 78 | 100 |
| ... with `--min-output 0.2` | 0 | 25 | 29 | 34 | 40 | 47 | 56 | 68 | 83 | 100 |

**The motors only hum below about 21 % (right) and 28 % (left) duty** (measured on the bench 2026-11-19,
[proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md)). With the defaults that is the first half of
the stick's travel. `--min-output 0.2` makes the first movement past the deadzone start at about 22 % duty,
which skips most of that band; the price is that the robot starts with a small jump. Try it on the stand
first. The proper fix for the dead band is on the robot and is still the owner's decision (proposal 0013).

## What you need

- **The controller:** an Xbox Wireless Controller that has Bluetooth (Xbox One S and later, Series X|S; the
  original 2013 Xbox One controller has no Bluetooth). Paired with the Mac: System Settings > Bluetooth, hold
  the small pairing button on the top edge of the controller until the Xbox logo flashes fast, then click
  Connect. Afterwards, the Xbox button alone reconnects it. It must show as **Connected**, not just listed.
- **On the Mac:** Python 3 and `pygame-ce` (the maintained fork of pygame; it installs on Python 3.14, which
  plain `pygame` did not when this was written). In a Terminal, from the repo root, once:

  ```bash
  python3 -m venv .venv-teleop
  ```

  ```bash
  .venv-teleop/bin/pip install "pygame-ce>=2.5.8"
  ```

  Checked with pygame-ce 2.5.8 (SDL 2.32.10) on Python 3.14.4. Do not install `pygame` and `pygame-ce` in the
  same environment: both provide `import pygame`.
- **On the Pi:** tank mode running, from `~/ROCKO-env`: `pi/launch.sh tank`
  ([Run ROCKO on the Pi](run-on-the-pi.md)). The Pi must have this change: in `~/ROCKO-env`, as your own user
  (not root), `git pull` on `ai-forward`, then `./build_ros.sh`. An older build ignores the shaped flag and curves
  the stick twice.
- Both machines on the same network, and the Pi's IP address (`hostname -I` on the Pi).
- **No other sender running.** `Joystick.py` accepts one at a time: quit `keyboard_teleop.py` or
  `computer_joystick.py` first.

## First: look at what the Mac sees (ten minutes, no robot)

Switch the controller on, then in a Terminal at the repo root:

```bash
.venv-teleop/bin/python joystick/controller_probe.py
```

Success looks like this (your GUID and mapping will differ):

```text
pygame 2.5.8, SDL 2.32.10
Waiting up to 5 s for a controller...
[0] 'Xbox Wireless Controller'  GUID 050000005e040000...  axes 6  buttons 16  GameController mapping: yes
    mapping: 050000005e04...,Xbox Wireless Controller,a:b0,b:b1,...
Move the sticks. Ctrl-C to stop.

left_y -0.98  right_x +0.00  LB 0 RB 1 Menu 0   raw axes: +0.01 -0.98 ...   attached True
```

Check, and note anything different in the [journal](../journal/2026-11-20-xbox-controller-split-arcade.md):

1. Left stick **up** gives `left_y` near **-1.00**, down near +1.00. Right stick **right** gives `right_x` near
   **+1.00**. If they are wrong or swapped, see *What goes wrong*.
2. Hands off: how far from 0.00 does each stick rest? If more than 0.10, raise `--deadzone` above it.
3. LB, RB and Menu read 1 when pressed.
4. Click a different window so the Terminal is behind it, and move a stick. The numbers should keep changing.

The `raw axes` column is the numbered view other programs use; it is there only so you can see that the
named values are the right sticks. This program never uses the numbers.

## Drive

**The first time, wheels off the ground** (the robot on a stand, as in the
[bench test](bench-test-tank-mode.md)), and with low caps. In a Terminal at the repo root on the Mac:

```bash
.venv-teleop/bin/python joystick/xbox_teleop.py <robot-ip> --max-speed 0.4 --max-turn 0.3
```

Success looks like:

```text
Controller: Xbox Wireless Controller
Connected to 192.168.1.42:1234. Hold LB or RB to drive; Menu or Ctrl-C quits.
DRIVE      forward   +38%  left    +0%  wheels L   +38% R   +38%
```

and the Pi's launch log prints `Joystick connected!`. The status line shows the command as a percent of full
wheel speed; motor duty is about that plus 2 %.

On the stand, check in this order, and **record the battery voltage and the commit** as
[reproducing results](reproducing-results.md) asks:

1. Nothing turns until a bumper is held.
2. Left stick up: both wheels turn the way that drives the robot forward. Right stick right: the left wheel
   forward and the right wheel back (a right turn).
3. Let go of the bumper with the stick held: both wheels slow to a stop (from full speed this may take up to
   about 1.5 s; see below).
4. Hold a bumper and the stick, then **switch the controller off** (hold the Xbox button about 6 s). The
   program should print `The controller disconnected. A stop was sent` and the wheels stop. Note how long it
   took.
5. Raise `--max-speed` in steps. Only then put the robot on the floor, with space around it.

## What stops the robot

| Event | What happens |
| --- | --- |
| Bumper released | A zero command is sent at once, then resent every 0.05 s. The robot's controller slows the wheels at its acceleration limit (forward: 10 m/s², so up to about 1.5 s from full speed, if it applies the same limit when slowing, which has not been checked; a turn stops almost at once) |
| Menu, Ctrl-C | A stop is sent, then the program exits |
| The controller turns off or SDL reports it gone | A zero command is sent (slowing as above), then the program exits (exit code 2) |
| The Mac, the program or the network dies | Nothing more arrives; tank mode's controller sets the command to zero after its `cmd_vel_timeout` of 1.0 s |

**Not covered:** a controller that drops off Bluetooth *without* macOS or SDL reporting it. The program would
keep resending the last values it read, bumper included, so the robot would keep driving and see a healthy
link; releasing the bumper would not help, because nothing from the controller reaches the Mac any more.
Then the only stops are Ctrl-C on the Mac or stopping the Pi's launch. Step 4 above is the test for this: if
the wheels do not stop when the controller is switched off, stop the robot that way and report it. Walking
out of Bluetooth range is the same case and has not been tried.

## What goes wrong

- **`No controller found`.** It is listed in Bluetooth but not *Connected*: press the Xbox button. If it will
  not connect, remove it in System Settings and pair again.
- **`A joystick is connected but SDL has no controller mapping for it`.** SDL does not recognise it as a
  standard controller. Run the probe and send the GUID line. A newer pygame-ce may know it.
- **The sticks are swapped, or a direction is reversed.** On macOS, SDL can reach the controller through two
  drivers: Apple's Game Controller framework and its own HIDAPI driver. Older firmware and older SDL versions
  have mixed up Xbox layouts over Bluetooth. Try each, one at a time, and run the probe again:

  ```bash
  SDL_JOYSTICK_HIDAPI=0 .venv-teleop/bin/python joystick/controller_probe.py
  ```

  ```bash
  SDL_JOYSTICK_MFI=0 .venv-teleop/bin/python joystick/controller_probe.py
  ```

  Whichever one fixes the probe, put the same variable in front of `xbox_teleop.py`. Updating the controller's
  firmware (Xbox Accessories app on Windows or an Xbox) also fixes old Bluetooth layouts.
- **Input stops when the Terminal is not the front window.** Both scripts set
  `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1` to prevent that; whether macOS honours it on your version is
  what step 4 of the probe checks. If it does not, keep the Terminal in front while driving.
- **`Refused a second joystick connection`** in the Pi's log. Another sender is still connected. Quit it, wait
  half a second, and start again.
- **The robot drops the connection** (`Rejected a gamepad message`). A packet had an axis outside [-1, 1];
  this program clamps every axis, so report it.
- **The wheels hum and do not turn at small stick.** Expected below about 21 to 28 % duty: see the table and
  `--min-output` above.

## Where the numbers live

| Number | Where |
| --- | --- |
| Deadzone, expo, minimum output, caps, poll period | `joystick/xbox_teleop.py`, the constants at the top |
| Full-range speed and turn in tank mode | `rocko_tank_mode.launch.py`, the `joystick` node |
| Which packet byte is which | `Joystick.py`: axis 1 forward, axis 0 turn, `SHAPED_BUTTON` 14, `PRECISE_BUTTON` 15 |
| The motor's speed-to-duty formula | `Motor12VoltQuadEncoder::write` |
