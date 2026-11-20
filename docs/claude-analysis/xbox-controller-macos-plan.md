# Plan: drive ROCKO with an Xbox controller over Bluetooth from macOS

- **Written:** 2026-11-19, by the agent, from reading the code. **Nothing here has been tried.** No
  controller was available to the agent. Statements about macOS, SDL and the Xbox controller are from
  general knowledge and are marked *to verify*.
- **Status:** **implemented 2026-11-20**, with two changes from this plan: the owner asked for split arcade
  (right stick X turns, not left stick X) and the full speed range, so the laptop shapes the sticks itself and the
  robot gained a shaped-axes flag; and the program uses SDL's GameController API (named axes) instead of joystick
  axis numbers. Use [the guide](../guides/drive-with-xbox-controller.md); the
  [journal](../journal/2026-11-20-xbox-controller-split-arcade.md) has the evidence. The rest of this page is the
  plan as written. It was next-steps row 4.5. Owner's request: "Xbox controller
  support via Bluetooth on macOS to drive the robot", before any self-balancing work.

## What exists

The gamepad path already works end to end, and the robot side needs **no change** for a standard stick
layout:

```text
Xbox controller --Bluetooth--> Mac: pygame joystick 0
    joystick/computer_joystick.py: 6 float32 axes + 16 uint8 buttons (40 bytes), resent every 0.05 s
        --TCP :1234--> Pi: Joystick.py
            axis 1 -> forward, axis 0 -> turn   (expo curve, deadband 0.05, scales -10 and -35)
            button 15 held -> "precise mode" (low-speed mapping)
        --> diffbot_base_controller/cmd_vel
```

`computer_joystick.py` is the old gamepad sender; `keyboard_teleop.py` imports its `pack` and
`HEARTBEAT_PERIOD_S`. What it does and does not do today, read from the file:

| Today | Consequence for an Xbox controller |
| --- | --- |
| The robot's IP is a constant (`ROCKO_IP`, line 6); no command-line argument | The address must be edited in the file for each network. The keyboard script takes it as an argument; this should too |
| Sends *every* axis and button pygame reports, padded or cut to 6 and 16 | Triggers and the right stick are sent too. `Joystick.py` reads only axes 0 and 1 and the button at index 15, so extra values are harmless **unless they are out of range**: the robot drops the connection when an axis is outside [-1, 1] |
| No deadzone | A stick that does not rest at exactly 0 sends small values. `Joystick.py` zeroes only below 0.05 after the expo curve. Xbox sticks commonly rest a few percent off centre (*to verify on this controller*) |
| No enable button | Touching the stick moves the robot |
| Runs `send_loop` forever with no handling of the controller disappearing | **The open safety question below** |
| The connect loop runs once; no reconnect | Restart the script after a drop |

## The safety question that decides the design

The robot's protection against a dead laptop is that packets stop arriving (finding C-3; in balancing mode
the watchdog zeroes the command after 0.2 s, in tank mode the controller's own `cmd_vel_timeout` of 1.0 s).
That protects against **the laptop or the script dying**. The new failure is different: **the controller
drops off Bluetooth while the script keeps running.** The script resends its last packet every 0.05 s, so
the robot would see a healthy link and keep the last stick position. What pygame does then (an error, a
frozen last value, zeros) *must be tested*, not assumed. The script has to treat "the controller is gone"
as "send a stop, then exit", whatever pygame does.

## Steps

Each step is small and says who does it.

1. **Probe (owner, ten minutes, no robot).** Pair the controller and print what pygame sees: name, number of
   axes and buttons, and live values. This is the one fact the rest depends on. The agent should write
   `joystick/controller_probe.py` (standard library plus pygame, about thirty lines) so the owner runs one
   command. Record the result in this page as a table: which index is left stick X and Y, the triggers, and
   A/B/X/Y, bumpers, Menu. Check: sticks go -1 to 1, up on the stick is negative (the robot's scales assume
   that, and `keyboard_teleop.py` imitates it), and a trigger's resting value (many report -1 at rest, not 0).
2. **Decide the mapping (owner, with the agent's recommendation).** Recommended: left stick Y = forward,
   left stick X = turn (indices 1 and 0, which need no `Joystick.py` change if the probe agrees); **hold a
   bumper to enable** (otherwise send zeros); hold the other bumper for slow mode (button 15 set, the
   existing precise mapping) until the owner chooses to drop it; Menu = stop. The speed ceiling is a robot-side
   number: today a full stick is the gamepad scale (about 10 "m/s", which is 67 % duty in tank mode with the 0.127 m wheel radius,
   a value next-steps row 0.7 says is disputed), which is a lot for a first run.
3. **Write the sender (agent).** A new `joystick/xbox_teleop.py` that reuses `pack` and `send_loop`, takes the
   robot address as an argument, applies a laptop-side deadzone (a named constant, a starting value of 0.1 to
   be set from the probe), the enable button, a clamp to [-1, 1] on every axis, and on controller loss sends
   a stop packet and exits non-zero. Keep `computer_joystick.py` working; do not grow it.
4. **Host test (agent, no controller).** Like `tools/computer_joystick_heartbeat_test.py`, which imports `send_loop` without pygame, with a fake clock;
   the fake `pygame` with scripted axes is new work. Cases: the mapping, the deadzone, the enable button zeroes
   everything, an out-of-range axis is clamped, a vanished controller sends a stop and exits. Make each test
   fail on a deliberate break before trusting it (CLAUDE.md section 4).
5. **Robot test (owner).** Wheels off the ground first, in tank mode:
   [the bench-test guide](../guides/bench-test-tank-mode.md) for the setup, then drive with the controller.
   Required cases: release the enable button (wheels stop); switch the controller off with the stick held
   (wheels stop within a second or so: **record how long**); walk out of Bluetooth range if the lab allows;
   close the laptop lid. Only then the floor.
6. **Docs (agent).** A guide `docs/guides/drive-with-xbox-controller.md` in the house format (which machine,
   what to run, what success looks like, what goes wrong), linked from the guides index and the README; the
   ladder's status updated; a journal entry.

## What to verify on macOS (from memory, not checked)

- **Which controllers pair.** The Bluetooth-capable Xbox controllers (Xbox One S and later, Series X|S)
  pair with macOS 11 or later from System Settings, Bluetooth, after holding the pairing button on top until the
  logo flashes fast. Older Xbox One controllers have no Bluetooth. Firmware may need updating from a
  Windows or Xbox console.
- **Does pygame see it.** pygame 2 uses SDL2, which should list a paired Xbox controller as joystick 0. The
  axis and button numbering on macOS can differ from Linux and Windows guides: use the probe, not a web page.
- **Input only while a window has focus.** SDL on macOS may deliver controller input only to a frontmost
  application. A script started from Terminal may need `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1` in its
  environment to keep receiving input when another window is in front. Test by clicking another window while
  the probe runs.
- **A window or not.** `pygame.event.pump()` with no display window is what the old script does; check it
  also works on the owner's macOS version and Python.
- **pygame's version.** The Python dependencies are unpinned (next-steps row 0.8, issue #14). Pin pygame when
  the working version is known, and record it.

## Order relative to the motor hum

The Xbox stick makes the motor hum worse until proposal 0013 is settled. A stick moves smoothly through
duty values from 0 up, and today everything below about 21 to 28 % only hums. A laptop-side deadzone does not
fix that: it removes the creep at rest, not the dead band inside the motor. So:

- Do steps 1 and 2 (probe, mapping) now; they need no robot.
- Decide proposal 0013 (a per-motor stiction duty, and a keyboard or stick scale for tank mode) before step 5,
  or the first robot test will feel like the keyboard did.
- A speed ceiling for a stick in tank mode needs the encoder velocities the bench test did not capture (at
  4.0 and 5.0 m/s): ask for them in the next bench run.

## What this plan does not cover

Balancing mode with a controller: a stick command is a velocity reference to the cascade, and nothing in
this plan was checked against the robot's balance behavior. That comes after the self-balancing work, behind
[proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md)'s tethered procedure. Windows and
Linux laptops were not considered (the keyboard script already works on macOS and Linux).
