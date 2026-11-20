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
    MAX_BYTES_PER_TICK bytes are read per tick;
  * Joystick's mappings: gamepad (expo, deadband), precise (keyboard) and shaped axes
    (joystick/xbox_teleop.py), and that the tank launch file's full-duty scales match the YAML.
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
NO_RAMP = 1e9  # [m/s^2] a ramp limit so large that a command applies in one cycle


def make_controller(timeout_param=None, accel_param=None):
    FakeNode.param_overrides = {}
    if timeout_param is not None:
        FakeNode.param_overrides["command_timeout_s"] = timeout_param
    if accel_param is not None:
        FakeNode.param_overrides["max_linear_accel_m_s2"] = accel_param
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
    # NO_RAMP: this test is about the watchdog, so commands apply at once (the ramp has its own tests).
    node, clock = make_controller(accel_param=NO_RAMP)
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


# --- the command ramp (finding H-13) --------------------------------------------------------------
def feedforwards(node):
    return node.publishers["left_feedforward"].sent[-1].data, node.publishers["right_feedforward"].sent[-1].data


def test_step_toward_moves_by_at_most_the_step_and_stops_on_the_target():
    check(diffdrive.step_toward(0.0, 1.0, 0.25) == 0.25, "up by one step")
    check(diffdrive.step_toward(0.0, -1.0, 0.25) == -0.25, "down by one step")
    check(diffdrive.step_toward(0.9, 1.0, 0.25) == 1.0, "lands exactly on a target closer than one step")
    check(diffdrive.step_toward(1.0, 1.0, 0.25) == 1.0, "stays on the target")


def test_forward_command_is_ramped_not_stepped():
    accel = diffdrive.DEFAULT_MAX_LINEAR_ACCEL_M_S2
    node, clock = make_controller()
    send_command(node, 0.5, 0.0)
    previous = 0.0
    for cycle in range(1, 400):
        cycles(node, clock, 1)
        send_command(node, 0.5, 0.0)  # the gamepad keeps sending, so the watchdog stays quiet
        value = last_reference(node)[0]
        check(value - previous <= accel * 0.01 + 1e-12, f"cycle {cycle}: rose by more than accel * 10 ms")
        check(value <= 0.5, f"cycle {cycle}: the ramp must not pass the command")
        previous = value
    check(previous == 0.5, "the ramp must arrive exactly at the command")
    left, right = feedforwards(node)
    check(left == right == 0.5 / node.wheel_radius_meters, "the feedforward follows the ramped value")
    # Reversing ramps down through zero at the same rate.
    send_command(node, -0.5, 0.0)
    cycles(node, clock, 1)
    check(abs(last_reference(node)[0] - (0.5 - accel * 0.01)) < 1e-12, "a reversal ramps down, it does not jump")


def test_first_cycle_of_a_command_is_one_ramp_step():
    node, clock = make_controller(accel_param=2.0)
    send_command(node, 1.0, 0.0)
    cycles(node, clock, 1)
    check(abs(last_reference(node)[0] - 0.02) < 1e-12, "2 m/s^2 for 10 ms is 0.02 m/s")


def test_timeout_is_a_step_to_zero_and_driving_resumes_from_zero():
    node, clock = make_controller(accel_param=1.0)
    for _ in range(50):  # 0.5 s of driving: the ramp reaches 0.5 m/s
        send_command(node, 0.5, 0.0)
        cycles(node, clock, 1)
    check(abs(last_reference(node)[0] - 0.5) < 1e-9, "set-up: ramp reached the command")
    cycles(node, clock, 25)  # silence, past the 0.2 s timeout
    check(last_reference(node) == [0.0, 0.0], "the watchdog's zero is immediate, not ramped")
    send_command(node, 0.5, 0.0)
    cycles(node, clock, 1)
    check(abs(last_reference(node)[0] - 0.01) < 1e-12, "after a time-out the ramp restarts from zero")


def test_turn_command_is_not_ramped():
    node, clock = make_controller()
    send_command(node, 0.0, 0.3)
    cycles(node, clock, 1)
    left, right = feedforwards(node)
    check(left == -0.3 / node.wheel_radius_meters and right == 0.3 / node.wheel_radius_meters,
          "the turn command reaches the feedforwards in the first cycle")
    check(last_reference(node) == [0.0, 0.0], "a pure turn leaves the velocity reference at zero")


def test_non_finite_command_is_ignored_and_does_not_feed_the_watchdog():
    node, clock = make_controller()
    for bad in (math.nan, math.inf, -math.inf):
        send_command(node, bad, 0.0)
        send_command(node, 0.0, bad)
    cycles(node, clock, 5)
    check(last_reference(node) == [0.0, 0.0], "a NaN or infinite command must not move the reference")
    check(feedforwards(node) == (0.0, 0.0), "a NaN or infinite command must not reach the feedforwards")
    check(not node.command_watchdog.has_received(), "a dropped command must not count as an arrival")
    # After a good command, a bad one leaves the good target in place (until the watchdog times out).
    send_command(node, 0.5, 0.0)
    send_command(node, math.nan, 0.0)
    cycles(node, clock, 1)
    check(last_reference(node)[0] > 0.0, "the last good command still applies after a bad one")


def test_bad_ramp_parameter_falls_back_to_default():
    for bad in (0.0, -1.0, math.inf, math.nan):
        node, _ = make_controller(accel_param=bad)
        check(node.max_linear_accel_m_s2 == diffdrive.DEFAULT_MAX_LINEAR_ACCEL_M_S2,
              f"bad ramp limit {bad} must fall back to DEFAULT_MAX_LINEAR_ACCEL_M_S2")
        check(any(level == "error" for level, _ in node.logger.lines), f"bad ramp limit {bad} must log an error")


# --- Joystick: gamepad mapping and precise mode (keyboard) ---------------------------------------
def twist_for(axes, buttons, shaped_scales=(joystick.Joystick.DEFAULT_SHAPED_LINEAR_SCALE_M_S,
                                         joystick.Joystick.DEFAULT_SHAPED_ANGULAR_SCALE)):
    """What Joystick.convert_to_twist publishes for one gamepad message: (linear.x, angular.z)."""
    js, gamepad = make_joystick_with_socket()
    gamepad.close()
    js.linear_scale, js.angular_scale, js.deadband = -10.0, -35.0, 0.05  # as Joystick.__init__ sets them
    js.precise_linear_scale = joystick.Joystick.DEFAULT_PRECISE_LINEAR_SCALE_M_S
    js.precise_angular_scale = joystick.Joystick.DEFAULT_PRECISE_ANGULAR_SCALE_M_S
    js.shaped_linear_scale, js.shaped_angular_scale = shaped_scales
    js.robot_body_vector_updated_topic = FakePublisher("cmd_vel")
    js.convert_to_twist(Msg(axes=axes, buttons=buttons))
    sent = js.robot_body_vector_updated_topic.sent[-1].twist
    return sent.linear.x, sent.angular.z


def test_precise_mode_uses_the_low_scales_with_no_deadband_or_expo():
    precise = [0] * 15 + [1]
    linear, angular = twist_for([0.0, -0.2, 0, 0, 0, 0], precise)  # one keyboard step forward
    check(abs(linear - 0.1) < 1e-9, f"one fifth of full axis must be 0.1 m/s forward, got {linear}")
    check(angular == 0.0, "no turn was asked for")
    linear, angular = twist_for([-1.0, -1.0, 0, 0, 0, 0], precise)  # full forward, full left
    check(abs(linear - 0.5) < 1e-9 and abs(angular - 0.3) < 1e-9, "full axis must be the precise scales")
    linear, angular = twist_for([0.02, 0.02, 0, 0, 0, 0], precise)
    check(linear != 0.0 and angular != 0.0, "precise mode has no deadband: a small axis is a small command")


def test_gamepad_mapping_is_unchanged_without_the_precise_button():
    gamepad_buttons = [0] * 16
    linear, angular = twist_for([0.0, -1.0, 0, 0, 0, 0], gamepad_buttons)
    check(abs(linear - 10.0) < 1e-9, "full stick is still 10 'm/s' for a gamepad")
    linear, _ = twist_for([0.0, -0.05, 0, 0, 0, 0], gamepad_buttons)
    check(linear == 0.0, "the gamepad's deadband still applies")
    # Buttons 0 to 13 do not switch the mode (14 is SHAPED_BUTTON, 15 is PRECISE_BUTTON).
    linear, _ = twist_for([0.0, -0.5, 0, 0, 0, 0], [1] * 14 + [0, 0])
    check(abs(linear - 10.0 * joystick.expo_curve(0.5)) < 1e-6,
          f"buttons 0-13 must leave the gamepad's expo mapping in place, got {linear}")


def test_precise_scale_parameter_is_bounded():
    js, gamepad = make_joystick_with_socket()
    gamepad.close()
    default = joystick.Joystick.DEFAULT_PRECISE_LINEAR_SCALE_M_S
    for value, expected in ((-0.2, -0.2), (-1.0, -1.0), (0.5, default), (0.0, default),
                            (-1.5, default), (math.nan, default)):
        js.get_parameter = lambda name, v=value: SimpleNamespace(value=v)
        got = js.precise_scale("precise_linear_scale_m_s", default)
        check(got == expected, f"precise scale {value} must give {expected}, got {got}")


# --- Joystick: shaped axes (joystick/xbox_teleop.py) ---------------------------------------------
SHAPED = [0] * 14 + [1, 0]  # button 14 set
TANK_SCALES = (-15.0, -15.0 / (0.32 / 2))  # what rocko_tank_mode.launch.py passes (checked below)


def test_shaped_axes_are_scaled_with_no_expo_or_deadband():
    linear, angular = twist_for([0.0, -0.5, 0, 0, 0, 0], SHAPED, TANK_SCALES)
    check(abs(linear - 7.5) < 1e-9, f"half axis must be half the scale (no second expo), got {linear}")
    check(angular == 0.0, "no turn was asked for")
    linear, angular = twist_for([-1.0, -1.0, 0, 0, 0, 0], SHAPED, TANK_SCALES)
    check(abs(linear - 15.0) < 1e-9 and abs(angular - 93.75) < 1e-9, "full axis must be the shaped scales")
    linear, angular = twist_for([0.02, 0.02, 0, 0, 0, 0], SHAPED, TANK_SCALES)
    check(linear != 0.0 and angular != 0.0, "no deadband here: the laptop already applied one")


def test_precise_mode_wins_over_shaped():
    linear, _ = twist_for([0.0, -1.0, 0, 0, 0, 0], [0] * 14 + [1, 1], TANK_SCALES)
    check(abs(linear - 0.5) < 1e-9, f"with both buttons set the slow precise scale must win, got {linear}")


def test_shaped_defaults_are_the_gamepad_scales():
    # Balance mode (rocko.launch.py) passes no shaped_* parameter, so a full shaped stick there must ask
    # for no more than a full gamepad stick does today (the -10 and -35 in Joystick.__init__).
    check(joystick.Joystick.DEFAULT_SHAPED_LINEAR_SCALE_M_S == -10.0, "default linear must be -10")
    check(joystick.Joystick.DEFAULT_SHAPED_ANGULAR_SCALE == -35.0, "default angular must be -35")


def test_shaped_scale_parameter_is_bounded():
    js, gamepad = make_joystick_with_socket()
    gamepad.close()
    default = joystick.Joystick.DEFAULT_SHAPED_LINEAR_SCALE_M_S
    limit = joystick.Joystick.MAX_SHAPED_LINEAR_SCALE_M_S
    for value, expected in ((-15.0, -15.0), (-0.1, -0.1), (0.0, default), (5.0, default),
                            (-15.1, default), (math.nan, default), (-math.inf, default)):
        js.get_parameter = lambda name, v=value: SimpleNamespace(value=v)
        got = js.shaped_scale("shaped_linear_scale_m_s", default, limit)
        check(got == expected, f"shaped scale {value} must give {expected}, got {got}")


def test_tank_launch_scales_match_this_test_and_the_yaml():
    # The launch file computes the scales from two numbers; one of them is copied from the YAML.
    import re
    launch = (REPO / "rocko_env/rocko_env/bringup/launch/rocko_tank_mode.launch.py").read_text()
    yaml = (REPO / "rocko_env/rocko_env/bringup/config/rocko_tank_mode_controllers.yaml").read_text()
    speed = float(re.search(r"full_duty_wheel_speed_m_s = ([0-9.]+)", launch).group(1))
    separation = float(re.search(r"wheel_separation_m = ([0-9.]+)", launch).group(1))
    yaml_separation = float(re.search(r"^\s*wheel_separation: ([0-9.]+)", yaml, re.M).group(1))
    check(separation == yaml_separation, f"launch wheel_separation_m {separation} != YAML {yaml_separation}")
    check((-speed, -speed / (separation / 2)) == TANK_SCALES, "TANK_SCALES here must match the launch file")
    check((-joystick.Joystick.MAX_SHAPED_LINEAR_SCALE_M_S, -joystick.Joystick.MAX_SHAPED_ANGULAR_SCALE)
          == TANK_SCALES, "the launch file's scales must be exactly Joystick's largest accepted ones")
    check("shaped_" not in (REPO / "rocko_env/rocko_env/bringup/launch/rocko.launch.py").read_text(),
          "the balancing launch file must not pass shaped_* parameters")


def test_xbox_teleop_packet_reaches_joystick_as_full_speed():
    # The laptop's packet, through the real receive path and mapping, with the tank scales.
    sys.path.insert(0, str(REPO / "joystick"))
    import xbox_teleop
    check(xbox_teleop.SHAPED_BUTTON == joystick.Joystick.SHAPED_BUTTON, "SHAPED_BUTTON differs between files")
    check(xbox_teleop.SHAPED_BUTTON != joystick.Joystick.PRECISE_BUTTON, "the two mode buttons must differ")
    js, gamepad = make_joystick_with_socket()
    gamepad.sendall(xbox_teleop.packet_for(1.0, 0.25))  # full forward, a quarter left
    message = js.read_newest_message()
    check(message is not None, "the laptop's packet must be accepted")
    linear, angular = twist_for(message.axes, message.buttons, TANK_SCALES)
    check(abs(linear - 15.0) < 1e-5, f"full forward must be 15 m/s in tank mode, got {linear}")
    check(abs(angular - 0.25 * 93.75) < 1e-4, f"a quarter left must be a positive (left) turn, got {angular}")


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
