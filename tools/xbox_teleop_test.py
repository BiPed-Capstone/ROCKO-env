#!/usr/bin/env python3
"""Host test for the Xbox controller sender (joystick/xbox_teleop.py). No ROS, no pygame, no controller.

Run from the repo root:  python3 -I tools/xbox_teleop_test.py

It checks the stick shaping and mixing as numbers, then runs drive() over a real socketpair with a
fake clock and a scripted controller:
  * shape(): zero inside the deadzone, no jump at its edge, +/-1 at full stick, the expo curve's value,
    odd symmetry, out-of-range input clamped, min_output lifts the start without changing full stick;
  * mix(): no wheel is ever asked for more than max_speed, and a turn survives at full forward;
  * command_from(): nothing moves without a bumper held; stick UP is forward, stick RIGHT is a right turn;
  * packets: 40 bytes, the shaped-axes button set and the precise one not, axes inside [-1, 1];
  * drive(): sends at once, heartbeats while nothing changes, stops at once when the bumper is released,
    and when the controller disappears or Menu is pressed the LAST packet is a stop.
What it cannot show: SDL, macOS, Bluetooth, what the real controller reports (joystick/controller_probe.py
on the laptop), what Joystick.py does with the packets (tools/teleop_watchdog_test.py), or the robot.
Not a tier.
"""
import itertools
import pathlib
import socket
import struct
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "joystick"))  # xbox_teleop imports computer_joystick from its own folder
import xbox_teleop as xt  # noqa: E402

FORMAT = f"{xt.NUM_AXES}f {xt.NUM_BUTTONS}B"
SIZE = struct.calcsize(FORMAT)
PRECISE_BUTTON = 15  # Joystick.PRECISE_BUTTON


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def unpack(packet):
    values = struct.unpack(FORMAT, packet)
    return list(values[:xt.NUM_AXES]), list(values[xt.NUM_AXES:])


# --- shape ---------------------------------------------------------------------------------------
def test_shape_deadzone_is_zero():
    for raw in (0.0, 0.05, -0.05, 0.1, -0.1):
        check(xt.shape(raw, deadzone=0.1, expo=0.6) == 0.0, f"{raw} is inside a 0.1 deadzone")


def test_shape_has_no_jump_at_the_deadzone_edge():
    just_outside = xt.shape(0.1001, deadzone=0.1, expo=0.0)
    check(0.0 < just_outside < 0.001, f"just outside the deadzone must be just above 0, got {just_outside}")


def test_shape_full_stick_is_full_command_whatever_the_settings():
    for deadzone, expo, min_output in itertools.product((0.0, 0.1, 0.3), (0.0, 0.6, 1.0), (0.0, 0.2)):
        check(close(xt.shape(1.0, deadzone, expo, min_output), 1.0), "full stick must be +1")
        check(close(xt.shape(-1.0, deadzone, expo, min_output), -1.0), "full stick back must be -1")


def test_shape_expo_value_and_symmetry():
    # deadzone 0: x = 0.5; (1 - 0.6) * 0.5 + 0.6 * 0.125 = 0.275
    check(close(xt.shape(0.5, deadzone=0.0, expo=0.6), 0.275), "expo 0.6 at half stick must be 0.275")
    check(close(xt.shape(-0.5, deadzone=0.0, expo=0.6), -0.275), "the curve must be odd-symmetric")
    check(close(xt.shape(0.5, deadzone=0.0, expo=0.0), 0.5), "expo 0 must be a straight line")
    check(xt.shape(0.5, 0.0, 0.6) < xt.shape(0.5, 0.0, 0.0), "expo must make half stick gentler")


def test_shape_clamps_out_of_range_input():
    check(close(xt.shape(1.7, 0.1, 0.6), 1.0) and close(xt.shape(-3.0, 0.1, 0.6), -1.0),
          "input outside [-1, 1] must be clamped")


def test_shape_min_output_lifts_the_start():
    check(close(xt.shape(0.1001, deadzone=0.1, expo=0.0, min_output=0.2), 0.2, tol=1e-3),
          "just outside the deadzone must start at min_output")
    check(xt.shape(0.05, deadzone=0.1, expo=0.0, min_output=0.2) == 0.0, "the deadzone is still zero")


# --- mix -----------------------------------------------------------------------------------------
def test_mix_never_asks_a_wheel_for_more_than_max_speed():
    values = [i / 10 for i in range(-10, 11)]
    for forward, left, max_speed in itertools.product(values, values, (1.0, 0.4)):
        f, l = xt.mix(forward, left, max_speed=max_speed, max_turn=1.0)
        for wheel in (f - l, f + l):
            check(abs(wheel) <= max_speed + 1e-9, f"mix({forward}, {left}) asks a wheel for {wheel}")


def test_mix_keeps_the_turn_at_full_forward():
    f, l = xt.mix(1.0, 0.5, max_speed=1.0, max_turn=1.0)
    check(close(f - l, 1.0 / 3.0) and close(f + l, 1.0), f"full forward + half left: wheels {f - l}, {f + l}")
    check(l > 0.0, "the turn must survive")


def test_mix_leaves_small_commands_alone():
    check(xt.mix(0.3, 0.2, 1.0, 1.0) == (0.3, 0.2), "a command inside the limits is unchanged")
    check(close(xt.mix(0.0, 1.0, 1.0, 0.5)[1], 0.5), "max_turn scales the turn")


# --- command_from and packets --------------------------------------------------------------------
SETTINGS = xt.Settings(deadzone=0.1, expo=0.6, min_output=0.0, max_speed=1.0, max_turn=1.0)


def test_nothing_moves_without_a_bumper():
    stick = xt.StickInput(left_y=-1.0, right_x=1.0, deadman=False)
    check(xt.command_from(stick, SETTINGS) == (0.0, 0.0), "no bumper held: zero")


def test_stick_up_is_forward_and_stick_right_is_a_right_turn():
    forward, left = xt.command_from(xt.StickInput(left_y=-1.0, deadman=True), SETTINGS)
    check(close(forward, 1.0) and left == 0.0, f"left stick up must be full forward, got {forward}, {left}")
    forward, left = xt.command_from(xt.StickInput(right_x=1.0, deadman=True), SETTINGS)
    check(forward == 0.0 and close(left, -1.0), f"right stick right must be a right turn, got {left}")
    forward, left = xt.command_from(xt.StickInput(left_y=-1.0, right_x=1.0, deadman=True), SETTINGS)
    check(close(forward, 0.5) and close(left, -0.5), "both full: scaled together by mix()")


def test_the_left_stick_x_and_right_stick_y_do_nothing():
    # Split arcade: only left Y and right X exist in StickInput, so nothing else can leak in.
    check(set(vars(xt.StickInput())) == {"left_y", "right_x", "deadman", "quit", "attached"},
          "StickInput must carry only the two split-arcade axes and the three flags")


def test_packet_layout_and_signs():
    packet = xt.packet_for(1.0, 0.25)
    check(len(packet) == SIZE == 40, "the gamepad packet is 40 bytes")
    axes, buttons = unpack(packet)
    check(close(axes[xt.FORWARD_AXIS], -1.0) and close(axes[xt.TURN_AXIS], -0.25),
          "forward and left are sent NEGATIVE, as a gamepad reports them")
    check(buttons[xt.SHAPED_BUTTON] == 1 and buttons[PRECISE_BUTTON] == 0, "shaped set, precise not")
    check(sum(buttons) == 1 and all(a == 0.0 for i, a in enumerate(axes) if i not in (0, 1)),
          "nothing else set")
    axes, _ = unpack(xt.packet_for(5.0, -5.0))
    check(all(-1.0 <= a <= 1.0 for a in axes), "axes must stay in [-1, 1] or the robot drops the link")


# --- drive ---------------------------------------------------------------------------------------
class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


def run_drive(script):
    """Run drive() with a scripted controller: script(n) returns the StickInput for the n-th read
    (n = 0, 1, ...). Returns (exit code, [(time, forward, left)] of every packet sent)."""
    sender, receiver = socket.socketpair()
    clock = FakeClock()
    reads = itertools.count()
    code = xt.drive(sender, lambda: script(next(reads)), SETTINGS, now=clock, sleep=clock.sleep)
    sender.close()
    data = b""
    while chunk := receiver.recv(65536):
        data += chunk
    receiver.close()
    check(len(data) % SIZE == 0, "only whole packets may be sent")
    packets = []
    for i in range(0, len(data), SIZE):
        axes, buttons = unpack(data[i:i + SIZE])
        packets.append((-axes[xt.FORWARD_AXIS], -axes[xt.TURN_AXIS]))
    return code, packets


def test_controller_lost_sends_stop_last_and_exits():
    def script(n):
        if n < 20:
            return xt.StickInput(left_y=-1.0, deadman=True)  # full forward
        return xt.StickInput(attached=False)
    code, packets = run_drive(script)
    check(code == xt.EXIT_CONTROLLER_LOST, f"a lost controller must exit with EXIT_CONTROLLER_LOST, got {code}")
    check(any(close(f, 1.0) for f, _ in packets), "full forward was sent while connected")
    check(packets[-1] == (0.0, 0.0), f"the last packet must be a stop, got {packets[-1]}")


def test_menu_sends_stop_last_and_exits_ok():
    code, packets = run_drive(lambda n: xt.StickInput(left_y=-1.0, deadman=True, quit=n >= 5))
    check(code == xt.EXIT_OK, "Menu must exit with EXIT_OK")
    check(packets[-1] == (0.0, 0.0), "the last packet must be a stop")


def test_releasing_the_bumper_stops_at_once_and_heartbeats_continue():
    # Reads 0-9: driving. 10-59: bumper released (0.5 s at 100 Hz). 60: Menu.
    def script(n):
        return xt.StickInput(left_y=-1.0, deadman=n < 10, quit=n >= 60)
    code, packets = run_drive(script)
    first_stop = next(i for i, p in enumerate(packets) if i > 0 and p == (0.0, 0.0))
    check(packets[first_stop - 1] != (0.0, 0.0), "the stop must follow a driving packet directly")
    stops_while_released = len(packets) - first_stop - 1  # minus Menu's final stop
    # 0.5 s of an unchanged command must still be heartbeated every HEARTBEAT_PERIOD_S (0.05 s).
    check(stops_while_released >= 9, f"heartbeats must continue while stopped, got {stops_while_released}")


def test_first_packet_is_a_stop():
    _, packets = run_drive(lambda n: xt.StickInput(quit=True))
    check(packets == [(0.0, 0.0), (0.0, 0.0)], f"start and quit packets must be stops, got {packets}")


# --- runner ----------------------------------------------------------------------------------------
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
