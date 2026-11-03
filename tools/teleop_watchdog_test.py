#!/usr/bin/env python3
"""Host test for the teleop watchdog (finding C-3). No ROS needed.

Run from the repo root:  python3 -I tools/teleop_watchdog_test.py

It loads DiffDriveController.py and Joystick.py with small stand-ins for rclpy and the
message types, injects a fake clock, and checks:
  * CommandWatchdog: age is measured from the last command's ARRIVAL, on a monotonic clock;
  * DiffDriveController: when the command is stale the velocity reference and feedforward
    are published as ZERO, publishing never stops, and logs / the Bool topic change only on
    a transition;
  * Joystick: only commands that really arrived are published, a clean disconnect does
    not crash it, a second TCP connection cannot replace a client that is still sending (only one
    silent for longer than CLIENT_SILENT_REPLACE_S), accept() errors are survived, and no more than
    MAX_BYTES_PER_TICK bytes are read per tick.
What it cannot show: anything about ROS timing, DDS, or a robot. Tier 0 only.
"""
import importlib.util
import math
import pathlib
import socket
import struct
import sys
import time
import types
from types import SimpleNamespace

REPO = pathlib.Path(__file__).resolve().parent.parent
CONTROLLERS = REPO / "rocko_env" / "rocko_env" / "controllers"


# --- stand-ins for the ROS modules the two nodes import ---------------------------------
class FakeLogger:
    def __init__(self):
        self.lines = []  # (level, text)

    def info(self, text):
        self.lines.append(("info", text))

    def warning(self, text, **kwargs):  # kwargs: throttle_duration_sec, as rclpy accepts
        self.lines.append(("warning", text))

    def error(self, text):
        self.lines.append(("error", text))


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    param_overrides = {}

    def __init__(self, name):
        self.logger = FakeLogger()
        self.publishers = {}
        self.params = {}

    def get_logger(self):
        return self.logger

    def declare_parameter(self, name, default):
        self.params[name] = self.param_overrides.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.params[name])

    def create_publisher(self, msg_type, topic, qos):
        self.publishers[topic] = FakePublisher(topic)
        return self.publishers[topic]

    def create_timer(self, period, callback):
        return None

    def create_subscription(self, msg_type, topic, callback, qos):
        return None


class Msg:  # a message that accepts any attribute, like ROS messages do
    def __init__(self, **fields):
        self.twist = SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
        self.data = 0.0
        self.axes = []
        self.buttons = []
        self.__dict__.update(fields)


def install_stubs():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod

    class RateLimiter:
        def __init__(self, **kwargs):
            pass

        def limit(self, *args):
            return 0.0

    module("rclpy")
    module("rclpy.node", Node=FakeNode)
    module("rclpy.qos", QoSProfile=lambda **kw: kw, DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="tl"))
    module("geometry_msgs")
    module("geometry_msgs.msg", TwistStamped=Msg, Twist=Msg, Vector3=Msg)
    module("control_msgs")
    module("control_msgs.msg", MultiDOFCommand=Msg)
    module("std_msgs")
    module("std_msgs.msg", Bool=Msg, Float64=Msg)
    module("sensor_msgs")
    module("sensor_msgs.msg", Joy=Msg)
    module("rocko_env")
    module("rocko_env.controllers")
    module("rocko_env.controllers.utils")
    module("rocko_env.controllers.utils.RateLimiter", RateLimiter=RateLimiter)


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


install_stubs()
diffdrive = load("DiffDriveController", CONTROLLERS / "diffdrive" / "DiffDriveController.py")
joystick = load("Joystick", CONTROLLERS / "joystick" / "Joystick.py")

TIMED_OUT_LOG = "teleop command timed out: zeroing the velocity reference, balance loop keeps running"
RESUMED_LOG = "teleop commands resumed"
NO_COMMAND_YET_LOG = "no teleop command yet: the velocity reference is zero"


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def check(condition, what):
    if not condition:
        raise AssertionError(what)


# --- CommandWatchdog --------------------------------------------------------------------
def test_before_first_command_is_expired():
    wd = diffdrive.CommandWatchdog(0.2, Clock())
    check(wd.expired(), "no command has ever arrived: must read as expired (reference is zero)")


def test_fresh_command_is_not_expired():
    clock = Clock()
    wd = diffdrive.CommandWatchdog(0.2, clock)
    wd.update()
    clock.t += 0.19
    check(not wd.expired(), "19 ms short of the timeout must not be expired")


def test_exactly_at_timeout_is_not_expired_just_after_is():
    clock = Clock()
    wd = diffdrive.CommandWatchdog(0.25, clock)  # 0.25 and 0.5 are exact in binary floating point
    wd.update()
    clock.t += 0.25
    check(not wd.expired(), "older than the timeout means strictly older: age == timeout is not expired")
    clock.t += 0.001
    check(wd.expired(), "just past the timeout must be expired")


def test_resumes_on_new_command():
    clock = Clock()
    wd = diffdrive.CommandWatchdog(0.2, clock)
    wd.update()
    clock.t += 1.0
    check(wd.expired(), "should have expired")
    wd.update()
    check(not wd.expired(), "a new command must clear the expiry")


def test_age_counts_from_arrival_not_from_checks():
    # The controller checks (and publishes) every 10 ms. Checking must never refresh the age.
    clock = Clock()
    wd = diffdrive.CommandWatchdog(0.2, clock)
    wd.update()
    for _ in range(50):  # 0.5 s of 10 ms cycles with no new command
        clock.t += 0.01
        wd.expired()
    check(wd.expired(), "repeated checks without a new command must not keep it alive")


def test_default_clock_is_monotonic():
    check(diffdrive.CommandWatchdog(0.2).now is time.monotonic, "must use time.monotonic, not time.time")


def test_nan_timeout_counts_as_expired():
    clock = Clock()
    wd = diffdrive.CommandWatchdog(math.nan, clock)
    wd.update()
    check(wd.expired(), "a NaN timeout must fail towards zero, not towards 'never expires'")


# --- DiffDriveController ----------------------------------------------------------------
def make_controller(timeout_param=None):
    FakeNode.param_overrides = {} if timeout_param is None else {"command_timeout_s": timeout_param}
    node = diffdrive.DiffDriveController()
    clock = Clock()
    node.command_watchdog.now = clock
    return node, clock


def last_reference(node):
    return node.publishers["velocity_pid_controller/reference"].sent[-1].values


def cycles(node, clock, n):
    for _ in range(n):
        clock.t += 0.01
        node.command_controller()


def send_command(node, linear_x, angular_z):
    node.robot_body_vector_updated(
        Msg(twist=SimpleNamespace(linear=SimpleNamespace(x=linear_x), angular=SimpleNamespace(z=angular_z))))


def test_node_zeroes_reference_and_feedforward_but_keeps_publishing():
    node, clock = make_controller()
    log = node.logger.lines
    bools = node.publishers["~/command_timed_out"].sent
    reference = node.publishers["velocity_pid_controller/reference"].sent
    left_ff = node.publishers["left_feedforward"].sent
    right_ff = node.publishers["right_feedforward"].sent

    # Startup: nobody is driving. Reference is zero and the state is reported once, as "no command
    # yet", NOT as the time-out warning (that one must mean the link went quiet).
    cycles(node, clock, 5)
    check(last_reference(node) == [0.0, 0.0], "startup reference must be zero")
    check([b.data for b in bools] == [True], "startup: exactly one Bool(True)")
    check(log.count(("info", NO_COMMAND_YET_LOG)) == 1, "startup: 'no teleop command yet' logged once")
    check(log.count(("warning", TIMED_OUT_LOG)) == 0, "startup must not log the time-out warning")

    # A command arrives: motion is passed through, 'resumed' logged once.
    send_command(node, 1.0, 0.5)
    cycles(node, clock, 5)
    check(last_reference(node) == [1.0, 1.0], "fresh command must reach the reference")
    check(left_ff[-1].data != 0.0 and right_ff[-1].data != 0.0, "fresh command must reach the feedforward")
    check([b.data for b in bools] == [True, False], "Bool must go True then False")
    check(log.count(("info", RESUMED_LOG)) == 1, "resumed line logged once")

    # 15 more cycles (150 ms) is still inside the 200 ms timeout.
    cycles(node, clock, 15)
    check(last_reference(node) == [1.0, 1.0], "inside the timeout the command still applies")

    # The link goes quiet. After the timeout everything that commands motion is zero ...
    published_before = len(reference)
    cycles(node, clock, 10)
    check(last_reference(node) == [0.0, 0.0], "stale command: reference must be zero")
    check(left_ff[-1].data == 0.0 and right_ff[-1].data == 0.0, "stale command: feedforward must be zero")
    check(log.count(("warning", TIMED_OUT_LOG)) == 1, "timed-out line logged once, not every cycle")
    cycles(node, clock, 50)
    check(log.count(("warning", TIMED_OUT_LOG)) == 1, "no repeat logging while timed out")
    check(log.count(("info", NO_COMMAND_YET_LOG)) == 1, "'no command yet' is for start-up only")
    check([b.data for b in bools] == [True, False, True], "Bool published only on changes")
    # ... but the node keeps publishing every cycle: the balance loop must keep being driven.
    check(len(reference) == published_before + 60, "the reference must keep being published while timed out")

    # The link returns.
    send_command(node, -0.5, 0.0)
    cycles(node, clock, 1)
    check(last_reference(node) == [-0.5, -0.5], "after the link returns the command applies again")
    check([b.data for b in bools] == [True, False, True, False], "Bool goes False again")
    check(log.count(("info", RESUMED_LOG)) == 2, "resumed line logged again")


def test_bad_timeout_parameter_falls_back_to_default():
    for bad in (0.0, -1.0, math.inf, math.nan):
        node, _ = make_controller(timeout_param=bad)
        check(node.command_watchdog.timeout_s == diffdrive.DEFAULT_COMMAND_TIMEOUT_S,
              f"bad timeout {bad} must fall back to DEFAULT_COMMAND_TIMEOUT_S")
        check(any(level == "error" for level, _ in node.logger.lines), f"bad timeout {bad} must log an error")


def test_custom_timeout_parameter_is_used():
    node, _ = make_controller(timeout_param=0.5)
    check(node.command_watchdog.timeout_s == 0.5, "the parameter must set the timeout")


# --- Joystick ---------------------------------------------------------------------------
def make_joystick_with_socket():
    """A Joystick without its TCP server: only the receive path, over a socketpair."""
    js = joystick.Joystick.__new__(joystick.Joystick)
    js.logger = FakeLogger()
    js.get_logger = lambda: js.logger
    js.message_size = struct.calcsize(f"{js.NUM_AXES}f {js.NUM_BUTTONS}B")
    js.rx_buffer = b""
    js.clock = Clock()
    js.now = js.clock
    js.last_rx_s = js.clock.t
    gamepad, robot_side = socket.socketpair()
    robot_side.setblocking(False)
    js.client_sock = robot_side
    return js, gamepad


def packet(axis1):
    return struct.pack("6f 16B", 0.0, axis1, 0.0, 0.0, 0.0, 0.0, *([0] * 16))


def test_joystick_returns_none_when_nothing_arrived():
    js, gamepad = make_joystick_with_socket()
    check(js.read_newest_message() is None, "no data: no message, so nothing is published")


def test_joystick_reads_newest_of_several_and_handles_split_packets():
    js, gamepad = make_joystick_with_socket()
    gamepad.sendall(packet(0.1) + packet(0.2) + packet(0.3)[:10])  # 2 whole + a partial one
    message = js.read_newest_message()
    check(abs(message.axes[1] - 0.2) < 1e-6, "must return the newest COMPLETE message")
    check(js.read_newest_message() is None, "the partial message is not complete yet")
    gamepad.sendall(packet(0.3)[10:])
    message = js.read_newest_message()
    check(abs(message.axes[1] - 0.3) < 1e-6, "the split message completes on the next read")


def test_default_timeout_is_four_heartbeats():
    # The robot's timeout and the laptop's heartbeat are set in two files; this keeps them in step.
    spec = importlib.util.spec_from_file_location("computer_joystick", REPO / "joystick" / "computer_joystick.py")
    cj = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cj)
    check(math.isclose(diffdrive.DEFAULT_COMMAND_TIMEOUT_S, 4 * cj.HEARTBEAT_PERIOD_S),
          "DEFAULT_COMMAND_TIMEOUT_S must be 4 x the laptop's HEARTBEAT_PERIOD_S")
    node, _ = make_controller()
    check(node.command_watchdog.timeout_s == diffdrive.DEFAULT_COMMAND_TIMEOUT_S,
          "with no parameter the node must use DEFAULT_COMMAND_TIMEOUT_S")


def raw_packet(axes, buttons):
    return struct.pack("6f 16B", *axes, *buttons)


def test_joystick_accepts_full_scale_axes():
    js, gamepad = make_joystick_with_socket()
    gamepad.sendall(raw_packet([1.0, -1.0, 1.005, 0.0, 0.0, 0.0], [1] + [0] * 15))
    message = js.read_newest_message()
    check(message is not None, "axes at +/-1 (and float rounding just past) must be accepted")
    check(js.client_sock is not None, "a good message must not drop the connection")


def test_joystick_rejects_garbage_and_drops_the_connection():
    bad_messages = {
        "NaN axis": raw_packet([0.0, math.nan, 0.0, 0.0, 0.0, 0.0], [0] * 16),
        "infinite axis": raw_packet([math.inf, 0.0, 0.0, 0.0, 0.0, 0.0], [0] * 16),
        "axis above 1": raw_packet([0.0, 1.5, 0.0, 0.0, 0.0, 0.0], [0] * 16),
        "axis below -1": raw_packet([0.0, 0.0, -2.0, 0.0, 0.0, 0.0], [0] * 16),
        "button 2": raw_packet([0.0] * 6, [2] + [0] * 15),
        "button 255": raw_packet([0.0] * 6, [0] * 15 + [255]),
    }
    for what, data in bad_messages.items():
        js, gamepad = make_joystick_with_socket()
        gamepad.sendall(data)
        check(js.read_newest_message() is None, f"{what}: the message must be rejected (nothing published)")
        check(js.client_sock is None, f"{what}: the connection must be dropped")
        check(any(level == "warning" and "Rejected" in text for level, text in js.logger.lines),
              f"{what}: the rejection must be logged")


def test_joystick_survives_a_clean_disconnect():
    js, gamepad = make_joystick_with_socket()
    gamepad.close()
    check(js.read_newest_message() is None, "a closed connection gives no message")
    check(js.client_sock is None, "the dead socket must be dropped so a new gamepad can connect")
    check(any(level == "warning" for level, _ in js.logger.lines), "the disconnect must be logged")
    check(js.read_newest_message() is None, "reading with no client must not raise")


class FakeServer:
    """Stands in for the listening socket: accept() returns the next queued socket, or raises."""

    def __init__(self, *outcomes):
        self.outcomes = list(outcomes)  # a socket to hand over, or an Exception instance to raise

    def accept(self):
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome, ("127.0.0.1", 9999)


def test_new_connection_does_not_replace_a_live_client():
    js, gamepad = make_joystick_with_socket()
    original = js.client_sock
    newcomer, newcomer_far_end = socket.socketpair()
    js.server_sock = FakeServer(newcomer)
    js.clock.t += 0.3  # the client sent 0.3 s ago: a heartbeat is 0.05 s, so it is alive
    js.accept_new_client()
    check(js.client_sock is original, "a live client must keep its place")
    check(newcomer.fileno() == -1, "the newcomer's socket must be closed")
    check(any(level == "warning" and "Refused" in text for level, text in js.logger.lines),
          "the refusal must be logged")
    gamepad.sendall(packet(0.4))
    check(abs(js.read_newest_message().axes[1] - 0.4) < 1e-6, "the original client must still be read")


def test_new_connection_replaces_a_silent_client_only_after_the_limit():
    js, gamepad = make_joystick_with_socket()
    original = js.client_sock
    js.rx_buffer = b"stale"
    newcomer, _ = socket.socketpair()
    js.server_sock = FakeServer(newcomer, newcomer)
    js.clock.t += js.CLIENT_SILENT_REPLACE_S  # 0.5 is exact in binary floating point
    js.accept_new_client()
    check(js.client_sock is original, "silent for exactly the limit is not LONGER than it: no replacement")
    newcomer, _ = socket.socketpair()
    js.server_sock = FakeServer(newcomer)
    js.clock.t += 0.001
    js.accept_new_client()
    check(js.client_sock is newcomer, "a client silent for longer than the limit must be replaced")
    check(original.fileno() == -1, "the old client's socket must be closed")
    check(js.rx_buffer == b"", "the old client's partial bytes must be dropped with it")
    check(js.last_rx_s == js.clock.t, "the new client starts its silence timer at the connection")


def test_first_connection_is_accepted():
    js, gamepad = make_joystick_with_socket()
    js.client_sock = None
    newcomer, _ = socket.socketpair()
    js.server_sock = FakeServer(newcomer)
    js.accept_new_client()
    check(js.client_sock is newcomer, "with no client the connection must be accepted")


def test_data_refreshes_the_silence_timer():
    js, gamepad = make_joystick_with_socket()
    js.clock.t += 0.4
    js.read_newest_message()  # nothing arrived
    check(js.last_rx_s == 1000.0, "reading nothing must not refresh the timer")
    gamepad.sendall(packet(0.1)[:7])  # even a fragment shows the client is alive
    js.read_newest_message()
    check(js.last_rx_s == js.clock.t, "bytes arriving must refresh the timer")


def test_accept_errors_do_not_crash_the_node():
    js, gamepad = make_joystick_with_socket()
    original = js.client_sock
    js.server_sock = FakeServer(ConnectionAbortedError("ECONNABORTED"), BlockingIOError())
    js.accept_new_client()  # must not raise
    js.accept_new_client()
    check(js.client_sock is original, "a failed accept() must leave the current client alone")
    check(any(level == "warning" and "accept() failed" in text for level, text in js.logger.lines),
          "an accept() error must be logged")


def test_bytes_read_per_tick_are_capped():
    js, gamepad = make_joystick_with_socket()
    gamepad.setblocking(False)
    sent = 0
    while sent < 3 * js.MAX_BYTES_PER_TICK:  # as much as the socket will take, up to three ticks' worth
        try:
            sent += gamepad.send(bytes(4096))  # zeros are a valid gamepad stream at any byte offset
        except BlockingIOError:
            break
    check(sent > js.MAX_BYTES_PER_TICK, f"the test could only queue {sent} bytes: too few to show the cap")
    js.read_newest_message()
    waiting = 0
    js.client_sock.setblocking(False)
    while True:  # count what is still waiting in the socket
        try:
            waiting += len(js.client_sock.recv(65536))
        except BlockingIOError:
            break
    check(sent - waiting == js.MAX_BYTES_PER_TICK,
          f"one tick must read exactly MAX_BYTES_PER_TICK bytes from a flood, it read {sent - waiting}")


# --- runner -----------------------------------------------------------------------------
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
