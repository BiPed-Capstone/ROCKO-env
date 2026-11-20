#!/usr/bin/env python3
"""Drive ROCKO in TANK MODE from an Xbox controller paired with this Mac over Bluetooth.
Runs ON THE LAPTOP, in a terminal. Needs Python 3 and pygame-ce (no ROS):

    python3 joystick/xbox_teleop.py <robot-ip>            # options: --help

SPLIT ARCADE: the LEFT stick, up and down, drives forward and back; the RIGHT stick, left and right,
turns. Hold LB or RB (either bumper) to drive: let go and the robot is told to stop. Press Menu
(the right-hand small button, "Start") or Ctrl-C to stop and quit.

How it reaches the robot: the gamepad's protocol (computer_joystick.py: 6 float32 axes and 16 uint8
buttons, 40 bytes, over TCP to Joystick.py on port 1234). Unlike computer_joystick.py, this script
shapes the sticks ITSELF (deadzone, expo curve, mixing, below) and sets Joystick.SHAPED_BUTTON so the
robot does not curve them a second time. What a full stick means in m/s is decided on the robot
(rocko_tank_mode.launch.py: 100 % motor duty). Guide: docs/guides/drive-with-xbox-controller.md.

macOS AND AXIS NUMBERS. A raw "joystick" in SDL (the library under pygame) numbers its axes in an
order that depends on the operating system, the driver and even the controller's firmware: on one
Mac the right stick's X can be axis 2, on another axis 3. So this script does NOT use axis numbers.
It uses SDL's GameController API, which looks the controller up in SDL's mapping database and
reports named axes (left stick Y, right stick X). joystick/controller_probe.py prints what SDL
sees, if anything here looks wrong.

What stops the robot: letting go of the bumpers (zero is sent); the controller disappearing (a stop
is sent, then the script exits); quitting; and, if this laptop or the network dies, the robot's own
timeout (tank mode's cmd_vel_timeout, 1.0 s). NOT covered: a controller that drops off Bluetooth
without SDL noticing would leave its last values in place. See the guide's "What stops the robot".
"""

import argparse
import os
import socket
import sys
import time
from dataclasses import dataclass

# SHAPED_BUTTON (Joystick.SHAPED_BUTTON): 1 = "these axes are already shaped, scale them only".
from computer_joystick import HEARTBEAT_PERIOD_S, NUM_AXES, NUM_BUTTONS, ROCKO_PORT, SHAPED_BUTTON, pack

# Which parts of the 40-byte packet the robot's Joystick.py reads. MUST match Joystick.py (written
# in both files because the laptop has no copy of the robot's code).
TURN_AXIS = 0  # Joystick.convert_to_twist: angular_axis
FORWARD_AXIS = 1  # Joystick.convert_to_twist: linear_axis

# --- the tunables a student would want to try (each is also a command-line option) ---------------
DEADZONE = 0.10  # [fraction of stick travel] stick values smaller than this count as zero. Sticks
#                  rarely rest at exactly 0; set it from controller_probe.py's resting values.
EXPO = 0.6  # [0..1] 0 = straight line, 1 = pure cube. More expo = finer control near the centre,
#             the same top speed at full stick.
MIN_OUTPUT = 0.0  # [fraction of full speed] the command just outside the deadzone. 0 = start from
#                   zero. The motors only hum below about 21 to 28 % duty (proposal 0013), so a
#                   value near 0.2 skips that band; leave 0 until the bench test says otherwise.
MAX_SPEED = 1.0  # [fraction of full wheel speed] cap on any wheel. 1.0 = 100 % motor duty.
MAX_TURN = 1.0  # [fraction] a full turn stick alone spins the wheels at this fraction of full.

POLL_PERIOD_S = 0.01  # [s] how often the controller is read (100 Hz); the robot's loop is 100 Hz too
STATUS_PERIOD_S = 0.1  # [s] how often the status line is redrawn
CONNECT_TIMEOUT_S = 5.0  # [s] how long to wait for the robot to accept the TCP connection
AXIS_FULL_SCALE = 32767  # SDL GameController sticks report -32768..32767; this maps them to [-1, 1]
CONTROLLER_WAIT_S = 5.0  # [s] how long to wait at start-up for macOS to report the controller

EXIT_OK = 0
EXIT_CONNECTION_LOST = 1
EXIT_CONTROLLER_LOST = 2


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, value))


def shape(raw, deadzone=DEADZONE, expo=EXPO, min_output=MIN_OUTPUT):
    """One stick axis in [-1, 1] -> a command in [-1, 1], same sign.
    1. Deadzone: |raw| <= deadzone is 0.
    2. Rescale what is left to 0..1, so the command starts at 0 at the deadzone's edge instead of
       jumping to `deadzone` (a jump would be a kick to the motors).
    3. Expo: (1 - expo) * x + expo * x^3. Same curve family as Joystick.expo_curve. At x = 1 it is
       still 1, so expo never costs top speed; it only spreads the slow speeds over more stick.
    4. Lift by min_output: min_output + (1 - min_output) * curve, still 1 at full stick.
    deadzone must be below 1 (main() checks the option), or step 2 divides by zero."""
    raw = clamp(raw)
    magnitude = abs(raw)
    if magnitude <= deadzone:
        return 0.0
    x = (magnitude - deadzone) / (1.0 - deadzone)
    curved = (1.0 - expo) * x + expo * x ** 3
    lifted = min_output + (1.0 - min_output) * curved
    return lifted if raw > 0 else -lifted


def mix(forward, turn_left, max_speed=MAX_SPEED, max_turn=MAX_TURN):
    """Combine forward and turn (each in [-1, 1]; forward > 0 drives forward, turn_left > 0 turns
    LEFT, which makes the right wheel faster) so that no wheel is asked for more than max_speed of
    full wheel speed. Returns (forward, turn_left), still as fractions of full.
    Each wheel's speed, as a fraction of full, is forward - turn_left (left wheel) and
    forward + turn_left (right wheel). This holds because the tank launch file scales turning so that
    a full turn alone gives the same wheel speed as a full forward. Order of the steps:
    1. max_turn weakens the turn stick first, so it is part of what the wheels must fit.
    2. Full forward plus some turn would ask one wheel for more than 1: the motor would clip at
       100 % and the turn would be lost. So when |forward| + |turn| > 1 both are divided by that sum,
       which keeps the turn and the direction of travel.
    3. max_speed scales both last, so it caps every wheel."""
    turn = turn_left * max_turn
    total = abs(forward) + abs(turn)
    if total > 1.0:
        forward, turn = forward / total, turn / total
    return forward * max_speed, turn * max_speed


def packet_for(forward, turn_left):
    """The 40-byte packet for a command (fractions of full; forward > 0 forward, turn_left > 0 left).
    SIGN: a gamepad reports "stick pushed forward" and "stick pushed left" as NEGATIVE values and
    Joystick.py's scales are negative to undo that, so forward and turn_left are sent negative here."""
    axes = [0.0] * NUM_AXES
    axes[FORWARD_AXIS] = clamp(-forward)
    axes[TURN_AXIS] = clamp(-turn_left)
    buttons = [0] * NUM_BUTTONS
    buttons[SHAPED_BUTTON] = 1
    return pack(axes, buttons)


STOP_PACKET = packet_for(0.0, 0.0)


class StickInput:
    """One reading of the controller. left_y and right_x are in [-1, 1] as SDL reports them: stick UP
    is NEGATIVE y, stick RIGHT is POSITIVE x."""

    def __init__(self, left_y=0.0, right_x=0.0, deadman=False, quit=False, attached=True):
        self.left_y = left_y
        self.right_x = right_x
        self.deadman = deadman  # True while LB or RB is held
        self.quit = quit  # True when Menu was pressed
        self.attached = attached  # False once the controller is gone


@dataclass
class Settings:
    """The tunables above, as one object (main() fills it from the command line)."""
    deadzone: float = DEADZONE
    expo: float = EXPO
    min_output: float = MIN_OUTPUT
    max_speed: float = MAX_SPEED
    max_turn: float = MAX_TURN


def command_from(stick, settings):
    """StickInput -> (forward, turn_left), fractions of full wheel speed. Zero unless a bumper is held.
    SIGNS, the whole chain: SDL reports stick UP as -1 and stick RIGHT as +1. This function turns them
    into forward > 0 and turn_left > 0 (the robot's own sense). packet_for() then flips both back to
    the gamepad convention (forward and left negative), and Joystick.py's negative scales flip them
    once more into positive m/s forward and positive (left) turn."""
    if not stick.deadman:
        return 0.0, 0.0
    shaping = (settings.deadzone, settings.expo, settings.min_output)
    forward = -shape(stick.left_y, *shaping)  # stick up (negative y) = forward
    turn_left = -shape(stick.right_x, *shaping)  # stick right (positive x) = a right turn
    return mix(forward, turn_left, settings.max_speed, settings.max_turn)


def drive(sock, read_stick, settings, now=time.monotonic, sleep=time.sleep, show=lambda *a: None):
    """Read the controller every POLL_PERIOD_S; send the packet when it changes and at least every
    HEARTBEAT_PERIOD_S (the robot treats silence as a dead link). Returns EXIT_OK when Menu is
    pressed and EXIT_CONTROLLER_LOST when the controller disappears; either way the last packet sent
    is a stop. read_stick, now, sleep and show are parameters so a test can run this without a
    controller, a clock or a terminal."""
    packet = STOP_PACKET
    sock.sendall(packet)
    last_sent_s = now()
    while True:
        stick = read_stick()
        if not stick.attached:
            sock.sendall(STOP_PACKET)
            return EXIT_CONTROLLER_LOST
        if stick.quit:
            sock.sendall(STOP_PACKET)
            return EXIT_OK
        forward, turn_left = command_from(stick, settings)
        show(stick, forward, turn_left)
        new_packet = packet_for(forward, turn_left)
        if new_packet != packet or now() - last_sent_s >= HEARTBEAT_PERIOD_S:
            packet = new_packet
            sock.sendall(packet)
            last_sent_s = now()
        sleep(POLL_PERIOD_S)


# --- the parts that need pygame and a real controller (not run by the host test) -----------------

def open_controller():
    """Start SDL's GameController API and return (pygame, controller module, the first controller).
    Exits with a message if none appears within CONTROLLER_WAIT_S."""
    # Keep receiving the controller while the Terminal is not the frontmost window. SDL's default
    # is to drop controller input for an app in the background, and on macOS pygame counts as an
    # app. Must be set before SDL starts. setdefault: an SDL_* variable you set yourself wins.
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame
    from pygame._sdl2 import controller as sdl_controller

    pygame.init()  # the event system SDL needs to see controllers comes with it
    sdl_controller.init()
    deadline = time.monotonic() + CONTROLLER_WAIT_S
    while time.monotonic() < deadline:
        pygame.event.pump()
        for index in range(pygame.joystick.get_count()):
            if sdl_controller.is_controller(index):
                return pygame, sdl_controller, sdl_controller.Controller(index)
        time.sleep(0.1)  # [s] macOS reports a controller within a second or so of SDL starting
    if pygame.joystick.get_count() > 0:
        sys.exit("A joystick is connected but SDL has no controller mapping for it. "
                 "Run joystick/controller_probe.py and see the guide.")
    sys.exit("No controller found. Switch it on (Xbox button) and check it shows as Connected in "
             "System Settings > Bluetooth, then try again.")


def stick_reader(pygame, sdl_controller, pad):
    """read_stick() for drive(), reading the real controller."""
    def axis(which):
        return clamp(pad.get_axis(which) / AXIS_FULL_SCALE)

    def read_stick():
        removed = False
        for event in pygame.event.get():  # also keeps SDL's queue from filling up
            if event.type == pygame.CONTROLLERDEVICEREMOVED:
                removed = True
        if removed or not pad.attached():
            return StickInput(attached=False)
        return StickInput(
            left_y=axis(pygame.CONTROLLER_AXIS_LEFTY),
            right_x=axis(pygame.CONTROLLER_AXIS_RIGHTX),
            deadman=bool(pad.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)
                         or pad.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)),
            quit=bool(pad.get_button(pygame.CONTROLLER_BUTTON_START)),
        )

    return read_stick


last_status_s = 0.0  # [s] when show_status last printed


def show_status(stick, forward, turn_left):
    """One status line, at most 10 times a second. The wheel numbers are percent of full wheel
    speed; the motor duty is roughly that plus the plugin's 2 %, not shown."""
    global last_status_s
    now_s = time.monotonic()
    if now_s - last_status_s < STATUS_PERIOD_S:
        return
    last_status_s = now_s
    mode = "DRIVE" if stick.deadman else "hold LB/RB"
    left_wheel, right_wheel = forward - turn_left, forward + turn_left  # see mix()
    sys.stdout.write(f"\r{mode:10s} forward {forward * 100:+5.0f}%  left {turn_left * 100:+5.0f}%  "
                     f"wheels L {left_wheel * 100:+5.0f}% R {right_wheel * 100:+5.0f}%   ")
    sys.stdout.flush()


def fraction(text):
    value = float(text)
    if not 0.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError(f"{text} is not between 0 and 1")
    return value


def main():
    parser = argparse.ArgumentParser(
        description="Drive ROCKO (tank mode) from an Xbox controller. Left stick: drive. "
                    "Right stick: turn. Hold LB or RB to move. Menu or Ctrl-C: stop and quit.")
    parser.add_argument("host", help="the robot's IP address or hostname")
    parser.add_argument("--port", type=int, default=ROCKO_PORT, help="Joystick.py's TCP port")
    parser.add_argument("--max-speed", type=fraction, default=MAX_SPEED,
                        help=f"cap on any wheel, fraction of full (default {MAX_SPEED})")
    parser.add_argument("--max-turn", type=fraction, default=MAX_TURN,
                        help=f"turn strength, fraction of full (default {MAX_TURN})")
    parser.add_argument("--expo", type=fraction, default=EXPO,
                        help=f"0 linear .. 1 cubic (default {EXPO})")
    parser.add_argument("--deadzone", type=fraction, default=DEADZONE,
                        help=f"stick travel that counts as zero (default {DEADZONE})")
    parser.add_argument("--min-output", type=fraction, default=MIN_OUTPUT,
                        help=f"command just outside the deadzone (default {MIN_OUTPUT})")
    args = parser.parse_args()
    if args.deadzone >= 1.0:
        parser.error("--deadzone must be below 1")
    settings = Settings(args.deadzone, args.expo, args.min_output, args.max_speed, args.max_turn)

    # The controller first: a missing controller should fail before anything touches the robot.
    pygame, sdl_controller, pad = open_controller()
    print(f"Controller: {pad.name}")
    sock = socket.create_connection((args.host, args.port), timeout=CONNECT_TIMEOUT_S)
    sock.settimeout(None)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # send each small packet at once
    print(f"Connected to {args.host}:{args.port}. Hold LB or RB to drive; Menu or Ctrl-C quits.")
    code = EXIT_OK
    try:
        code = drive(sock, stick_reader(pygame, sdl_controller, pad), settings, show=show_status)
        if code == EXIT_CONTROLLER_LOST:
            print("\nThe controller disconnected. A stop was sent; exiting.")
    except KeyboardInterrupt:
        try:
            sock.sendall(STOP_PACKET)
        except OSError:
            pass  # the robot's timeout stops it 1.0 s after our last packet
    except OSError as error:
        print(f"\nLost the connection to the robot ({error}).")
        code = EXIT_CONNECTION_LOST
    finally:
        sock.close()
        pygame.quit()
        print("\nStopped.")
    sys.exit(code)


if __name__ == "__main__":
    main()
