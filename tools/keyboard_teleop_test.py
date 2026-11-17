#!/usr/bin/env python3
"""Host test for the laptop keyboard sender (joystick/keyboard_teleop.py). No ROS, no terminal, no robot.

Run from the repo root:  python3 -I tools/keyboard_teleop_test.py

It checks the key handling (KeyboardState) and runs drive() over a real socketpair with a fake clock
and scripted keys:
  * w/s/a/d move the levels one step and stop at +/-STEPS_TO_FULL; space zeroes both; q zeroes and quits;
  * an arrow key (three bytes starting with ESC) is not mistaken for the letter it ends in;
  * every packet is the gamepad's 40 bytes, with the precise-mode button set and axes inside [-1, 1];
  * forward and left are sent as NEGATIVE axis values, as a gamepad reports them;
  * a key press is sent at once, an unchanged state is resent every HEARTBEAT_PERIOD_S, and the last
    packet before drive() returns is an all-stop.
What it cannot show: a real terminal, network timing, what Joystick.py does with the packets (that is
tools/teleop_watchdog_test.py and the sim check KB), or anything on the robot. Not a tier.
"""
import pathlib
import socket
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "joystick"))  # keyboard_teleop imports computer_joystick from its own folder
import keyboard_teleop as kt  # noqa: E402

FORMAT = f"{kt.NUM_AXES}f {kt.NUM_BUTTONS}B"
SIZE = struct.calcsize(FORMAT)


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def unpack(packet):
    values = struct.unpack(FORMAT, packet)
    return list(values[:kt.NUM_AXES]), list(values[kt.NUM_AXES:])


def pressed(keys):
    state = kt.KeyboardState()
    for key in keys:
        state.press(key)
    return state


def run_drive(script, step_s=0.01):
    """Run drive() with a fake clock that advances step_s per read_key call. script maps the
    number of the read_key call (1, 2, ...) to the key returned then. Returns [(time, packet)]."""
    sender, receiver = socket.socketpair()
    clock = {"t": 0.0, "calls": 0}
    sent = []

    class RecordingSocket:
        def sendall(self, data):
            sent.append((clock["t"], data))
            sender.sendall(data)

    def read_key(timeout_s):
        clock["calls"] += 1
        clock["t"] += step_s
        return script.get(clock["calls"])

    kt.drive(RecordingSocket(), read_key, now=lambda: clock["t"])
    sender.close()
    data = b""
    while True:
        chunk = receiver.recv(65536)
        if not chunk:
            break
        data += chunk
    receiver.close()
    check(len(data) == SIZE * len(sent), "every send must be exactly one 40-byte packet")
    return sent


def test_keys_step_the_levels_and_stop_at_full():
    state = pressed("ww")
    check((state.forward_level, state.turn_level) == (2, 0), "w is one step forward")
    state = pressed("w" * 20)
    check(state.forward_level == kt.STEPS_TO_FULL, "forward stops at full")
    state = pressed("s" * 20)
    check(state.forward_level == -kt.STEPS_TO_FULL, "backward stops at full")
    state = pressed("aaad")
    check(state.turn_level == 2, "a is one step left, d one step right")
    state = pressed("W")
    check(state.forward_level == 1, "capital letters work (caps lock)")
    state = pressed("wz?")
    check((state.forward_level, state.turn_level, state.quit) == (1, 0, False), "unknown keys do nothing")


def test_space_stops_and_q_stops_and_quits():
    state = pressed("wwa ")
    check((state.forward_level, state.turn_level, state.quit) == (0, 0, False), "space zeroes both levels")
    for quit_key in ("q", "\x04"):
        state = pressed("wwa" + quit_key)
        check((state.forward_level, state.turn_level, state.quit) == (0, 0, True), "quit zeroes and quits")


def test_packet_is_a_gamepad_packet_in_precise_mode():
    axes, buttons = unpack(pressed("wa").packet())
    check(len(pressed("").packet()) == 40, "the packet is the gamepad's 40 bytes")
    check(buttons[kt.PRECISE_BUTTON] == 1 and sum(buttons) == 1, "only the precise-mode button is set")
    check(abs(axes[kt.FORWARD_AXIS] + 1 / kt.STEPS_TO_FULL) < 1e-6, "forward is a NEGATIVE axis value, one step")
    check(abs(axes[kt.TURN_AXIS] + 1 / kt.STEPS_TO_FULL) < 1e-6, "left is a NEGATIVE axis value, one step")
    axes, _ = unpack(pressed("w" * 20 + "d" * 20).packet())
    check(all(-1.0 <= value <= 1.0 for value in axes), "axes stay inside [-1, 1] (Joystick.py rejects more)")
    axes, _ = unpack(pressed("").packet())
    check(axes == [0.0] * kt.NUM_AXES, "nothing pressed is all zeros")


def test_drive_sends_changes_at_once_heartbeats_and_ends_with_a_stop():
    # read_key call 30 returns "w" (t = 0.30 s), call 60 returns "q".
    sent = run_drive({30: "w", 60: "q"})
    times = [t for t, _ in sent]
    gaps = [b - a for a, b in zip(times, times[1:])]
    check(max(gaps) <= kt.HEARTBEAT_PERIOD_S + 0.011, f"no gap longer than a heartbeat (+1 step), got {max(gaps)}")
    check(len(sent) >= 0.6 / kt.HEARTBEAT_PERIOD_S, "an unchanged state is resent as a heartbeat")
    forward = [unpack(p)[0][kt.FORWARD_AXIS] for _, p in sent]
    first_moving = next(i for i, value in enumerate(forward) if value != 0.0)
    check(abs(times[first_moving] - 0.30) < 1e-9, "the key press is sent in the same loop pass, not at the next heartbeat")
    check(forward[-1] == 0.0 and unpack(sent[-1][1])[0][kt.TURN_AXIS] == 0.0, "the last packet is an all-stop")
    check(all(value == 0.0 for value in forward[:first_moving]), "nothing moves before the first key")


def test_arrow_keys_are_ignored_not_read_as_letters():
    import os
    reader, writer = os.pipe()
    try:
        os.write(writer, b"\x1b[A")  # up-arrow: ESC [ A. Read byte by byte, "A" would be the a key
        check(kt.read_key_from(reader, 0.5) is None, "an arrow key must report no key")
        check(kt.read_key_from(reader, 0.05) is None, "nothing of the arrow's sequence may be left behind")
        os.write(writer, b"w")
        check(kt.read_key_from(reader, 0.5) == "w", "ordinary keys still arrive")
        check(kt.read_key_from(reader, 0.05) is None, "no key, no result")
    finally:
        os.close(reader)
        os.close(writer)


def main():
    tests = [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as error:  # noqa: BLE001 - report every failure, not just the first
            failed += 1
            print(f"FAIL  {name}: {error!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
