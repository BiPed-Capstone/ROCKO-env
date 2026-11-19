# Drive the robot from a laptop keyboard

Use this when there is no gamepad, or when you want slower and more repeatable commands than a stick
gives. The keyboard script speaks the gamepad's protocol, so the robot needs nothing extra running.

**What has been checked:** the script and the robot-side code were run against the simulated robot
(check `KB` in [the verification ladder](../architecture/verification-ladder.md)) and in a
pseudo-terminal on a laptop. **Nothing here has been run on the robot.** Whether the robot stays
upright while driven is not known; the first time, follow the tethered procedure in
[proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md) and have a person holding
the tether.

**Tank mode (`rocko_tank_mode.launch.py`) is not covered by that check.** The keyboard's speeds were
sized for the balancing robot. In tank mode the top key level (0.5 m/s) is only about 5 % motor power,
which on the first run made the motors hum without turning the wheels. Bench-test it first, wheels off
the ground: [bench-test-tank-mode.md](bench-test-tank-mode.md); the decision about what to change is
[proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md).

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

There are five steps each way. What a step means is decided on the robot, in `Joystick.py`
(`DEFAULT_PRECISE_LINEAR_SCALE_M_S` and `DEFAULT_PRECISE_ANGULAR_SCALE_M_S`, the speed at a full axis;
both are **proposed** values nobody has tried on the robot; as parameters they are refused above
`MAX_PRECISE_SCALE_M_S`). The laptop cannot ask for more than
those. The robot also ramps the forward command (finding H-13), so a key press takes effect over a
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

The script keeps two numbers, a forward level and a turn level, each from -5 to +5. Every 0.05 s, and
at once when a key changes them, it sends the gamepad's 40-byte packet with those levels as two axis
values and one button byte set to mean "precise mode". `Joystick.py` sees that byte and, instead of
the gamepad's mapping (an expo curve, a deadband, and up to 10 "m/s"), multiplies the axis by a small
scale. From there the command takes the gamepad's path: `DiffDriveController.py` ramps it and sends it
to the balance cascade ([control-cascade.md](../architecture/control-cascade.md)).

The tests: `tools/keyboard_teleop_test.py` (the keys and the sending loop) and
`tools/teleop_watchdog_test.py` (`Joystick.py`'s two mappings); what each proves is in
[the ladder](../architecture/verification-ladder.md).
