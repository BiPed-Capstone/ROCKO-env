# Drive the robot from a laptop keyboard

Use this when there is no gamepad, or when you want slower and more repeatable commands than a stick
gives. The keyboard script speaks the gamepad's protocol, so the robot needs nothing extra running.

**What has been checked:** the script and the robot-side code were run against the simulated robot
(check `KB` in [the verification ladder](../architecture/verification-ladder.md)) and in a
pseudo-terminal on a laptop. **On 2026-11-19 the owner drove the robot in tank mode with it** (reported,
not recorded: [journal](../journal/2026-11-19-first-robot-run-tank-mode.md)). Whether the *balancing*
robot stays upright while driven is not known; the first time, follow the tethered procedure in
[proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md) and have a person holding
the tether.

> **Current state, and a mismatch (2026-11-19).** The owner's commit `df91b9e` turned **precise mode
> off** (the script no longer sets button 15) and made the keys **ten steps** each way (`STEPS_TO_FULL`
> = 10). The keyboard therefore uses the *gamepad's* mapping on the robot: an expo curve, a deadband,
> and up to 10 "m/s" (table below). Parts of this guide below still describe precise mode and five
> steps; where they differ, the table below and the code are right. Two checks still encode the old
> design and are **red**: `tools/keyboard_teleop_test.py` (4 of 5; the precise-button check) and
> scenario `KB` (it expects steps of 0.1 m/s). They stay red until the keyboard's design is settled
> ([next-steps row 4.7](../next-steps.md)).

**Tank mode (`rocko_tank_mode.launch.py`) is not covered by the simulation check.** With precise mode
on, the top key level (0.5 m/s) was only about 5 % motor power and the motors hummed without turning the
wheels (the reason precise mode was turned off). Bench-test first, wheels off the ground:
[bench-test-tank-mode.md](bench-test-tank-mode.md); what the owner measured and what is still undecided
is [proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md).

**What each key level does now** (gamepad mapping: `expo_curve(level / 10) x 10 m/s`, then in tank mode
duty = `6.5957 x speed + 2`, truncated; arithmetic, not a measurement; the first three speeds were
seen in the simulation, check `KB`):

| Level | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Speed [m/s] | 0.55 | 1.14 | 1.77 | 2.49 | 3.31 | 4.27 | 5.39 | 6.70 | 8.23 | 10.0 |
| Motor duty [%] | 5 | 9 | 13 | 18 | 23 | 30 | 37 | 46 | 56 | 67 |

On the owner's bench the left wheel first turned at 28 % duty and the right at 21 %, so levels 1 to 4
only hum, level 5 turns the right wheel alone, and level 6 and up turn both: the robot spends the low
levels in the "humming, close to stalling" region ([proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md)).

## What you need

- **On the robot (the Pi):** the normal launch, `pi/launch.sh balance` (or `pi/launch.sh tank`; see
  [Run ROCKO on the Pi](run-on-the-pi.md): the launch runs as root). It starts
  `Joystick.py`, which listens on TCP port 1234.
- **On your laptop:** Python 3 (nothing else: no ROS, no pygame), a copy of this repo's `joystick/`
  folder, and a terminal. Keep `keyboard_teleop.py` next to `computer_joystick.py`: it imports the
  packet format from it. macOS and Linux work. On Windows use WSL: the script needs a Unix terminal.
- Both machines on the same network, and the Pi's IP address (`hostname -I` on the Pi).
- **The gamepad script must not be running.** `Joystick.py` accepts one sender at a time and refuses a
  second while the first is alive.

## Drive

On the **laptop**, from the repo root (no `source` needed; this is not a ROS program):

```bash
python3 joystick/keyboard_teleop.py <robot-ip>
```

(`--port N` if `Joystick.py` has been changed to listen somewhere other than 1234.)

Success looks like:

```text
Connected to 192.168.1.42:1234.  w/s: forward/back   a/d: left/right   space: stop   q: quit
forward +0/5   left +0/5
```

and, on the Pi, the launch log prints `Joystick connected!` and then `teleop commands resumed`.

| Key | What it does |
| --- | --- |
| `w` / `s` | one step faster forward / backward |
| `a` / `d` | one step more left / right |
| space | stop: zero speed and zero turn |
| `q` (or Ctrl-C) | stop, then quit |
| arrow keys, anything else | nothing |

**Keys set a speed; they are not "hold to drive".** A terminal reports key presses but never key
releases, so the script cannot tell that you let go. The speed you set stays until you change it,
press space, or quit. Keep a finger near the space bar.

There are ten steps each way. What a step means is decided on the robot, in `Joystick.py`. With
precise mode on (button 15 set; **off since `df91b9e`**) it is `DEFAULT_PRECISE_LINEAR_SCALE_M_S` and
`DEFAULT_PRECISE_ANGULAR_SCALE_M_S`, the speed at a full axis (**proposed** values; as parameters they are
refused above `MAX_PRECISE_SCALE_M_S`, so the laptop cannot ask for more). With it off, the gamepad
mapping above applies and the cap does not. The robot also ramps the forward command (finding H-13), so a key press takes effect over a
moment, not at once.

## What stops the robot if something goes wrong

- **You quit, or press space:** the script sends a stop.
- **The laptop, the script or the network dies:** the script resends its packet every 0.05 s. When
  the robot hears nothing for 0.2 s, its teleop watchdog commands zero speed and keeps balancing
  (finding C-3). That stop is a step, not a ramp; see
  [proposal 0012](../proposals/0012-command-ramp-and-watchdog-step.md).
- **The robot falls:** the tipover cutoff stops the wheels (finding H-4). The keyboard cannot re-arm it.

None of these was checked on the robot.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| `ConnectionRefusedError` or a 5 s wait and `timed out` | The launch is not running, the IP is wrong, or the machines are on different networks | `ping <robot-ip>`; check the launch is up |
| It connects and closes at once; the Pi logs `Refused a second joystick connection` | The gamepad script (or another keyboard) is still connected | Stop the other sender, wait about a second (the Pi replaces a sender only after it has been silent for 0.5 s), start again |
| `needs a terminal` | The script was started through a pipe or an IDE's output pane | Run it in a real terminal |
| Keys print letters and nothing moves | The terminal window does not have focus, or caps lock plus a remapped layout | Click the terminal; the status line must change when you press `w` |
| The robot stops by itself after a moment, and the Pi logs `teleop command timed out` | Packets are arriving more than 0.2 s apart: a weak Wi-Fi link | Move closer; the stop is the watchdog doing its job |
| After a crash of the script the terminal does not echo what you type | The script could not restore the terminal | Type `reset` and press Enter |

## How it works, in four sentences

The script keeps two numbers, a forward level and a turn level, each from -10 to +10. Every 0.05 s, and
at once when a key changes them, it sends the gamepad's 40-byte packet with those levels as two axis
values and one button byte that says "precise mode" (set until `df91b9e`, clear now). With the byte set,
`Joystick.py` multiplies the axis by a small scale; with it clear, it applies the gamepad's mapping (an
expo curve, a deadband, and up to 10 "m/s"). From there the command takes the gamepad's path: `DiffDriveController.py` ramps it and sends it
to the balance cascade ([control-cascade.md](../architecture/control-cascade.md)).

The tests: `tools/keyboard_teleop_test.py` (the keys and the sending loop) and
`tools/teleop_watchdog_test.py` (`Joystick.py`'s two mappings); what each proves is in
[the ladder](../architecture/verification-ladder.md).
