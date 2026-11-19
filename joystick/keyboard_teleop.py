#!/usr/bin/env python3
"""Drive ROCKO from a laptop keyboard. Runs ON THE LAPTOP, in a terminal (macOS or Linux; on
Windows use WSL). Needs only Python 3: no ROS, no pygame.

    python3 joystick/keyboard_teleop.py <robot-ip>

    w / s    one step faster forward / backward        a / d    one step more left / right
    space    stop (zero speed and zero turn)            q        stop, then quit (Ctrl-C too)
    Arrow keys and other special keys do nothing.

How it reaches the robot: it speaks the gamepad's protocol (computer_joystick.py: 6 float32 axes and
16 uint8 buttons, 40 bytes, over TCP to Joystick.py on port 1234), so the robot treats it as a
gamepad and the same teleop watchdog guards it (finding C-3). Only one sender can be connected at a
time: stop computer_joystick.py first.

Why keys change the speed in STEPS instead of "hold to drive": a terminal reports key presses but
never key releases, so the script cannot tell that you let go. The speed you set stays until you
change it, press space, or quit.

What stops the robot if this laptop or the network dies: the packet below is resent every
HEARTBEAT_PERIOD_S; when the robot hears nothing for 0.2 s it commands zero speed by itself.
"""

import argparse
import os
import select
import socket
import sys
import time

from computer_joystick import (
    HEARTBEAT_PERIOD_S,
    NUM_AXES,
    NUM_BUTTONS,
    ROCKO_PORT,
    pack,
)

# Which parts of the gamepad packet the robot's Joystick.py reads. MUST match Joystick.py; they are
# written in both files because the laptop has no copy of the robot's code. The simulation check
# KB (verify_sim.sh scenarios KB) fails if the two drift apart.
TURN_AXIS = 0  # Joystick.convert_to_twist: angular_axis
FORWARD_AXIS = 1  # Joystick.convert_to_twist: linear_axis
PRECISE_BUTTON = 15  # Joystick.PRECISE_BUTTON: 1 = "precise mode", see Joystick.py

# Each w/s/a/d press moves the level by one; the axis sent is level / STEPS_TO_FULL, so it stays in
# [-1, 1]. What speed a full axis means is decided ON THE ROBOT (Joystick.py's precise_* parameters),
# so the laptop cannot ask for more than the robot allows.
STEPS_TO_FULL = 10

HELP = "w/s: forward/back   a/d: left/right   space: stop   q: quit"


class KeyboardState:
    """The speed and turn levels the keys have set, each an integer in [-STEPS_TO_FULL, STEPS_TO_FULL].
    forward_level > 0 means forward; turn_level > 0 means LEFT (counter-clockwise seen from above)."""

    def __init__(self):
        self.forward_level = 0
        self.turn_level = 0
        self.quit = False

    def press(self, key):
        """Apply one key. Unknown keys do nothing."""
        key = key.lower()
        if key == "w":
            self.forward_level = min(self.forward_level + 1, STEPS_TO_FULL)
        elif key == "s":
            self.forward_level = max(self.forward_level - 1, -STEPS_TO_FULL)
        elif key == "a":
            self.turn_level = min(self.turn_level + 1, STEPS_TO_FULL)
        elif key == "d":
            self.turn_level = max(self.turn_level - 1, -STEPS_TO_FULL)
        elif key == " ":
            self.forward_level = 0
            self.turn_level = 0
        elif key in (
            "q",
            "\x04",
        ):  # q, Ctrl-D. (Ctrl-C never arrives as a key: see main().)
            self.forward_level = 0
            self.turn_level = 0
            self.quit = True

    def packet(self):
        """The 40-byte gamepad packet for the current levels.
        SIGN: a gamepad reports "stick pushed forward" and "stick pushed left" as NEGATIVE axis
        values (pygame's convention), and Joystick.py's scales are negative to undo that. The
        keyboard imitates the gamepad, so forward and left are sent negative too."""
        axes = [0.0] * NUM_AXES
        axes[FORWARD_AXIS] = -self.forward_level / STEPS_TO_FULL
        axes[TURN_AXIS] = -self.turn_level / STEPS_TO_FULL
        buttons = [0] * NUM_BUTTONS
        buttons[PRECISE_BUTTON] = 0
        return pack(axes, buttons)


def drive(sock, read_key, state=None, now=time.monotonic, show=lambda state: None):
    """Send the state's packet when a key changes it and at least every HEARTBEAT_PERIOD_S, until
    the state says quit; then send one all-stop packet. read_key(timeout_s) returns one key, or None
    if none was pressed within timeout_s. read_key, now and show are parameters so a test (or the
    simulation check) can run this without a terminal."""
    state = state or KeyboardState()
    packet = state.packet()
    sock.sendall(packet)
    last_sent_s = now()
    show(state)
    while not state.quit:
        key = read_key(max(0.0, HEARTBEAT_PERIOD_S - (now() - last_sent_s)))
        if key is not None:
            state.press(key)
            show(state)
        new_packet = state.packet()
        if new_packet != packet or now() - last_sent_s >= HEARTBEAT_PERIOD_S:
            packet = new_packet
            sock.sendall(packet)
            last_sent_s = now()
    # quit zeroed the levels: make sure the robot's last word from us is "stop".
    sock.sendall(state.packet())


def read_key_from(fd, timeout_s):
    """One key from the terminal (file descriptor fd), or None if none arrives within timeout_s.

    os.read on the file descriptor, NOT sys.stdin.read: sys.stdin keeps its own buffer, so two keys
    typed close together would both be taken from the terminal, the second would wait unseen in
    that buffer, and select() (which only sees the terminal) would not report it until yet another
    key was pressed. A "q" could be held back that way.

    Arrow keys and function keys arrive as several bytes starting with ESC (up-arrow is ESC [ A).
    Read one byte at a time, that "A" would count as the "a" key and turn the robot. So on ESC the
    rest of the sequence is thrown away and no key is reported."""
    ready, _, _ = select.select([fd], [], [], timeout_s)
    if not ready:
        return None
    key = os.read(fd, 1).decode(errors="ignore")
    if key == "\x1b":
        while select.select([fd], [], [], 0.01)[
            0
        ]:  # the rest follows within a millisecond
            if not os.read(fd, 16):
                break  # the terminal closed
        return None
    return key


def terminal_key_reader():
    """Returns (read_key, restore) for the terminal this script runs in. The terminal is put in
    "cbreak" mode so a key arrives without Enter; restore() MUST be called before exiting."""
    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    return (lambda timeout_s: read_key_from(fd, timeout_s)), lambda: termios.tcsetattr(
        fd, termios.TCSANOW, saved
    )


def show_levels(state):
    sys.stdout.write(
        f"\rforward {state.forward_level:+d}/{STEPS_TO_FULL}   left {state.turn_level:+d}/{STEPS_TO_FULL}   "
    )
    sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(
        description="Drive ROCKO from the keyboard. " + HELP
    )
    parser.add_argument("host", help="the robot's IP address or hostname")
    parser.add_argument(
        "--port", type=int, default=ROCKO_PORT, help="Joystick.py's TCP port"
    )
    args = parser.parse_args()
    if not sys.stdin.isatty():
        sys.exit(
            "keyboard_teleop.py needs a terminal: run it directly, not through a pipe."
        )

    sock = socket.create_connection((args.host, args.port), timeout=5.0)
    sock.settimeout(None)
    sock.setsockopt(
        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
    )  # send each small packet at once
    print(f"Connected to {args.host}:{args.port}.  {HELP}")
    read_key, restore_terminal = terminal_key_reader()
    try:
        drive(sock, read_key, show=show_levels)
    except KeyboardInterrupt:
        # Ctrl-C. In cbreak mode the terminal turns it into this exception instead of a key, so
        # drive() never saw a "quit". Send the stop ourselves; if that fails too, the robot's
        # watchdog still zeroes the speed 0.2 s after our last packet.
        try:
            sock.sendall(KeyboardState().packet())
        except OSError:
            pass
    except (BrokenPipeError, ConnectionError) as error:
        # Nothing more can be sent. The robot's watchdog zeroes the speed 0.2 s after our last packet.
        print(f"\nLost the connection to the robot ({error}).")
    finally:
        restore_terminal()
        sock.close()
        print("\nStopped.")


if __name__ == "__main__":
    main()
