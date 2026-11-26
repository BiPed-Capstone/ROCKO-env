#!/usr/bin/env python3
"""Host test for the teleop watchdog (finding C-3). No ROS needed.

Run from the repo root:  python3 -I tools/teleop_watchdog_test.py

It loads DiffDriveController.py with small stand-ins for rclpy and the message types, injects
a fake clock, and checks:
  * CommandWatchdog: age is measured from the last command's ARRIVAL, on a monotonic clock;
  * DiffDriveController: when the command is stale the velocity reference and feedforward
    are published as ZERO, publishing never stops, and logs / the Bool topic change only on
    a transition;
  * the command ramp (finding H-13): a forward command is ramped, a turn is not, a bad
    command or parameter is refused.
DiffDriveController.py is balancing mode's; the gamepad side is tested by tools/balance_teleop_test.py
(balancing) and tools/tank_teleop_test.py (tank).
What it cannot show: anything about ROS timing, DDS, or a robot. Tier 0 only.
"""
import importlib.util
import math
import pathlib
import sys
import time
import types
from types import SimpleNamespace

REPO = pathlib.Path(__file__).resolve().parent.parent
CONTROLLERS = REPO / "rocko_env" / "rocko_env" / "controllers"


# --- stand-ins for the ROS modules the node imports ---------------------------------
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


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


install_stubs()
diffdrive = load("DiffDriveController", CONTROLLERS / "diffdrive" / "DiffDriveController.py")

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


def test_default_timeout_is_used_without_a_parameter():
    node, _ = make_controller()
    check(node.command_watchdog.timeout_s == diffdrive.DEFAULT_COMMAND_TIMEOUT_S,
          "with no parameter the node must use DEFAULT_COMMAND_TIMEOUT_S")


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
