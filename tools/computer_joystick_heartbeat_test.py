#!/usr/bin/env python3
"""Host test for the laptop gamepad sender's heartbeat (joystick/computer_joystick.py, finding C-3).
No ROS, no pygame, no gamepad needed.

Run from the repo root:  python3 -I tools/computer_joystick_heartbeat_test.py

It runs send_loop() over a real socketpair with a fake clock and a fake gamepad, and checks:
  * a held, unchanging stick is resent, byte for byte, every HEARTBEAT_PERIOD_S (not just once);
  * a change is sent at once, without waiting for the heartbeat;
  * every packet is still 40 bytes for 6 axes and 16 buttons (the protocol Joystick.py reads).
What it cannot show: real network timing, or anything on the robot. Tier 0 only.
"""
import importlib.util
import pathlib
import socket
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("computer_joystick", REPO / "joystick" / "computer_joystick.py")
cj = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cj)  # the script part is behind `if __name__ == "__main__"`, so nothing runs

AXES, BUTTONS = 6, 16


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def run(script, cycles, step_s):
    """Run send_loop for `cycles` loop iterations, the fake clock advancing step_s per iteration.
    script(i) gives the gamepad packet at iteration i. Returns the packets the robot side received."""
    sender, receiver = socket.socketpair()
    clock = FakeClock()
    count = {"i": 0}

    def keep_going():
        count["i"] += 1
        clock.t += step_s
        return count["i"] <= cycles

    cj.send_loop(sender, cj.pack([0.0] * AXES, [0] * BUTTONS), lambda: script(count["i"]),
                 now=clock, keep_going=keep_going)
    sender.close()
    data = b""
    while True:
        chunk = receiver.recv(65536)
        if not chunk:
            break
        data += chunk
    receiver.close()
    size = struct.calcsize(f"{AXES}f {BUTTONS}B")
    assert len(data) % size == 0, f"received {len(data)} bytes, not a whole number of {size}-byte packets"
    return [data[i:i + size] for i in range(0, len(data), size)], size


def test_packet_is_40_bytes():
    assert len(cj.pack([0.0] * AXES, [0] * BUTTONS)) == 40


def test_pack_pads_and_truncates_to_6_axes_16_buttons():
    # A gamepad with 4 axes and 10 buttons: padded with zeros.
    short = cj.pack([0.5, -0.5, 0.25, 1.0], [1] * 10)
    assert len(short) == 40, f"a small gamepad's packet must still be 40 bytes, got {len(short)}"
    values = struct.unpack("6f 16B", short)
    assert list(values[:6]) == [0.5, -0.5, 0.25, 1.0, 0.0, 0.0], f"axes not zero-padded: {values[:6]}"
    assert list(values[6:]) == [1] * 10 + [0] * 6, f"buttons not zero-padded: {values[6:]}"
    # A gamepad with 8 axes and 20 buttons: the extra ones are dropped.
    long = cj.pack([0.5] * 6 + [0.9, 0.9], [0] * 16 + [1] * 4)
    assert len(long) == 40, f"a big gamepad's packet must still be 40 bytes, got {len(long)}"
    values = struct.unpack("6f 16B", long)
    assert list(values[:6]) == [0.5] * 6 and list(values[6:]) == [0] * 16, "extra axes/buttons not cut"


def test_held_stick_is_resent_every_heartbeat():
    held = cj.pack([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], [0] * BUTTONS)
    # 1 s of a 1 ms loop with the stick held from the first iteration.
    packets, size = run(lambda i: held, cycles=1000, step_s=0.001)
    assert size == 40
    assert packets[0] == cj.pack([0.0] * AXES, [0] * BUTTONS), "the first packet is 'nothing pressed'"
    assert packets[1] == held, "the change is sent at once"
    resent = packets[2:]
    assert all(p == held for p in resent), "a heartbeat resends the SAME bytes"
    # 1 s / 0.05 s = 20 heartbeats (19 or 20 with the fake clock's rounding).
    assert 19 <= len(resent) <= 20, f"expected ~20 heartbeats in 1 s, got {len(resent)}"


def test_change_is_sent_at_once():
    a = cj.pack([0.1] + [0.0] * 5, [0] * BUTTONS)
    b = cj.pack([0.2] + [0.0] * 5, [0] * BUTTONS)
    # Changes every 10 ms, much faster than the heartbeat: every change must go out.
    packets, _ = run(lambda i: a if (i // 10) % 2 == 0 else b, cycles=100, step_s=0.001)
    changes = sum(1 for x, y in zip(packets, packets[1:]) if x != y)
    assert changes >= 10, f"expected one send per change (>= 10), got {changes}"


def test_no_heartbeat_inside_the_period():
    held = cj.pack([0.0] * AXES, [1] + [0] * (BUTTONS - 1))
    packets, _ = run(lambda i: held, cycles=40, step_s=0.001)  # 40 ms < 50 ms
    assert len(packets) == 2, f"expected the first packet and the change only, got {len(packets)}"


def main():
    tests = [test_packet_is_40_bytes, test_pack_pads_and_truncates_to_6_axes_16_buttons,
             test_held_stick_is_resent_every_heartbeat,
             test_change_is_sent_at_once, test_no_heartbeat_inside_the_period]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
