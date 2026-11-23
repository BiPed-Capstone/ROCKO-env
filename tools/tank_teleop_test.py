#!/usr/bin/env python3
"""Host test for tank mode's USB-controller teleop. No ROS, no controller, no robot.

Run from the repo root:  python3 -I tools/tank_teleop_test.py

It checks the shared module rocko_env/teleop/gamepad.py as plain functions, then loads
controllers/tank_teleop/TankTeleop.py with small stand-ins for rclpy and the message types and a fake
clock, and feeds it /joy messages:
  * gamepad: the /joy layout (LB = 9, left Y = 1, right X = 2), the 0.25 s silence timeout; shape():
    deadzone, no jump at its edge, full stick = 1, expo value, symmetry, clamping, min_output; mix(): no
    wheel above max_speed, the turn survives at full forward; DeadmanSwitch: needs a release first,
    and again after any silence;
  * TankTeleop: zero and disabled with no /joy, with RB instead of LB, before LB has been released once,
    after JOY_TIMEOUT_S of silence and after a reconnect that still reads LB held, for a message too
    short to hold LB; a NaN or infinite stick is a zero command; full left stick up = 15 m/s forward, right stick right = a
    right turn (negative angular.z), and the two together keep both wheels within full speed; bad
    parameters fall back to the defaults; step() publishes a stamped command and the enabled flag;
  * wiring: WHEEL_SEPARATION_M equals the tank YAML's; the tank launch starts game_controller_node and
    TankTeleop.py (not Joystick.py) with the joy node's rate taken from gamepad.py; the balancing launch
    starts neither, and balancing's nodes do not import the tank node.
What it cannot show: the joy node, SDL, the 8BitDo, ROS timing, diff_drive_controller, the robot.
Not a tier.
"""
import importlib.util
import itertools
import math
import pathlib
import re
import sys
import types
from types import SimpleNamespace

REPO = pathlib.Path(__file__).resolve().parent.parent
PACKAGE = REPO / "rocko_env"   # holds the importable rocko_env/ Python package
sys.path.insert(0, str(PACKAGE))
from rocko_env.teleop import gamepad  # noqa: E402  (pure Python: no ROS needed)


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


# --- gamepad.py ----------------------------------------------------------------------------------
def test_layout_and_timeout():
    check(gamepad.BUTTON_LEFT_BUMPER == 9 and gamepad.BUTTON_RIGHT_BUMPER == 10, "bumpers are buttons 9 and 10")
    check(gamepad.AXIS_LEFT_Y == 1 and gamepad.AXIS_RIGHT_X == 2, "left Y is axis 1, right X is axis 2")
    check(len(gamepad.BUTTON_NAMES) == 21 and len(gamepad.AXIS_NAMES) == 6, "joy 3.3.0: 21 buttons, 6 axes")
    check(close(gamepad.JOY_TIMEOUT_S, 0.25), "silence timeout is five 20 Hz messages")


def test_shape():
    check(all(gamepad.shape(r, 0.1, 0.6) == 0.0 for r in (0.0, 0.05, -0.1)), "inside the deadzone is 0")
    edge = gamepad.shape(0.1001, 0.1, 0.0)
    check(0.0 < edge < 0.001, f"no jump at the deadzone edge, got {edge}")
    for deadzone, expo, lift in itertools.product((0.0, 0.1, 0.3), (0.0, 0.6, 1.0), (0.0, 0.2)):
        check(close(gamepad.shape(1.0, deadzone, expo, lift), 1.0), "full stick is 1")
        check(close(gamepad.shape(-1.0, deadzone, expo, lift), -1.0), "full stick back is -1")
    check(close(gamepad.shape(0.5, 0.0, 0.6), 0.275) and close(gamepad.shape(-0.5, 0.0, 0.6), -0.275),
          "expo 0.6 at half stick is 0.275, odd-symmetric")
    check(close(gamepad.shape(3.0, 0.1, 0.6), 1.0), "out-of-range input is clamped")
    check(close(gamepad.shape(0.1001, 0.1, 0.0, 0.2), 0.2, tol=1e-3), "min_output lifts the start")


def test_mix():
    values = [i / 10 for i in range(-10, 11)]
    for forward, turn, cap in itertools.product(values, values, (1.0, 0.4)):
        f, t = gamepad.mix(forward, turn, cap, 1.0)
        check(max(abs(f - t), abs(f + t)) <= cap + 1e-9, f"mix({forward}, {turn}) asks a wheel for too much")
    f, t = gamepad.mix(1.0, 0.5)
    check(close(f + t, 1.0) and t > 0.0, "a turn at full forward survives, outer wheel at full")
    check(gamepad.mix(0.3, 0.2) == (0.3, 0.2), "a command inside the limits is unchanged")


def test_deadman_needs_a_release_first_and_after_silence():
    d = gamepad.DeadmanSwitch()
    check(d.update(True, True) is False, "held from the start: not enabled")
    check(d.update(True, False) is False and d.update(True, True) is True, "release, then hold: enabled")
    check(d.update(False, True) is False, "silence: disabled")
    check(d.update(True, True) is False, "after silence, still reading held: not enabled")
    check(d.update(True, False) is False and d.update(True, True) is True, "release and hold again: enabled")


# --- TankTeleop.py, with stand-ins for ROS ----------------------------------------------------------
class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text, **_):
        self.lines.append(("info", text))

    def error(self, text, **_):
        self.lines.append(("error", text))


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    param_overrides = {}

    def __init__(self, name):
        self.logger = FakeLogger()
        self.publishers = {}

    def get_logger(self):
        return self.logger

    def declare_parameter(self, name, default, descriptor=None):
        return SimpleNamespace(value=self.param_overrides.get(name, default))

    def create_subscription(self, msg_type, topic, callback, qos):
        return None

    def create_publisher(self, msg_type, topic, qos):
        self.publishers[topic] = FakePublisher()
        return self.publishers[topic]

    def create_timer(self, period, callback):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))


class Msg:
    def __init__(self, **fields):
        self.header = SimpleNamespace(stamp=None)
        self.twist = SimpleNamespace(linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
        self.data = None
        self.axes, self.buttons = [], []
        self.__dict__.update(fields)


def load_tank_teleop():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
    module("rclpy", executors=SimpleNamespace(ExternalShutdownException=Exception))
    module("rclpy.executors", ExternalShutdownException=Exception)
    module("rclpy.node", Node=FakeNode)
    module("rcl_interfaces")
    module("rcl_interfaces.msg", ParameterDescriptor=lambda **kw: kw)
    module("geometry_msgs")
    module("geometry_msgs.msg", TwistStamped=Msg)
    module("sensor_msgs")
    module("sensor_msgs.msg", Joy=Msg)
    module("std_msgs")
    module("std_msgs.msg", Bool=Msg)
    path = PACKAGE / "rocko_env" / "controllers" / "tank_teleop" / "TankTeleop.py"
    spec = importlib.util.spec_from_file_location("TankTeleop", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


tt = load_tank_teleop()


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make_node(params=None):
    FakeNode.param_overrides = params or {}
    node = tt.TankTeleop()
    node.now = Clock()
    return node


def joy(node, left_y=0.0, right_x=0.0, lb=0, rb=0, n_buttons=21):
    axes = [0.0] * 6
    axes[gamepad.AXIS_LEFT_Y], axes[gamepad.AXIS_RIGHT_X] = left_y, right_x
    buttons = [0] * n_buttons
    if n_buttons > gamepad.BUTTON_RIGHT_BUMPER:
        buttons[gamepad.BUTTON_LEFT_BUMPER], buttons[gamepad.BUTTON_RIGHT_BUMPER] = lb, rb
    node.on_joy(Msg(axes=axes, buttons=buttons))


def armed_node():
    """A node that has heard the controller with LB released once, so holding LB now drives."""
    node = make_node()
    joy(node)
    node.command()
    return node


def test_no_joy_is_zero_and_disabled():
    check(make_node().command() == (0.0, 0.0, False), "no /joy at all: zero, disabled")


def test_full_forward_and_turn_directions():
    node = armed_node()
    joy(node, left_y=1.0, lb=1)
    linear, angular, enabled = node.command()
    check(enabled and close(linear, 15.0) and angular == 0.0, f"left stick up = 15 m/s forward, got {linear}")
    joy(node, right_x=-1.0, lb=1)
    linear, angular, _ = node.command()
    check(linear == 0.0 and close(angular, -93.75), f"right stick right = right turn (-z), got {angular}")
    joy(node, right_x=1.0, lb=1)
    check(close(node.command()[1], 93.75), "right stick left = left turn (+z)")


def test_forward_plus_turn_keeps_both_wheels_within_full():
    node = armed_node()
    joy(node, left_y=1.0, right_x=-1.0, lb=1)
    linear, angular, _ = node.command()
    half_track = tt.WHEEL_SEPARATION_M / 2
    for wheel in (linear - angular * half_track, linear + angular * half_track):
        check(abs(wheel) <= tt.FULL_WHEEL_SPEED_M_S + 1e-9, f"a wheel was asked for {wheel} m/s")


def test_only_the_left_bumper_enables():
    node = armed_node()
    joy(node, left_y=1.0, rb=1)
    check(node.command() == (0.0, 0.0, False), "RB must not enable driving")


def test_lb_held_from_start_needs_a_release():
    node = make_node()
    joy(node, left_y=1.0, lb=1)
    check(node.command()[2] is False, "LB held from the first message: disabled")
    joy(node, left_y=1.0, lb=0)
    node.command()
    joy(node, left_y=1.0, lb=1)
    check(node.command()[2] is True, "after a release, LB enables")


def test_silence_stops_and_reconnect_needs_a_release():
    node = armed_node()
    joy(node, left_y=1.0, lb=1)
    check(node.command()[2] is True, "driving")
    node.now.t += gamepad.JOY_TIMEOUT_S - 0.01
    check(node.command()[2] is True, "just inside the timeout: still driving")
    node.now.t += 0.02
    check(node.command() == (0.0, 0.0, False), "past the timeout with no /joy: zero")
    joy(node, left_y=1.0, lb=1)  # the joy node after a replug, LB still reading held
    check(node.command()[2] is False, "a reconnect that reads LB held must not drive")
    check(any("controller gone" in text for _, text in node.logger.lines), "the silence is logged")


def test_short_message_and_nan_do_not_drive():
    node = armed_node()
    joy(node, left_y=1.0, n_buttons=8)  # too short to hold LB (index 9)
    check(node.command()[2] is False, "a message without an LB entry must not drive")
    node = armed_node()
    joy(node, left_y=math.nan, right_x=math.inf, lb=1)
    linear, angular, _ = node.command()
    check(linear == 0.0 and angular == 0.0, f"a NaN or infinite stick must give zero, got {linear}, {angular}")
    check(gamepad.shape(math.nan, 0.1, 0.6) == 0.0, "shape(NaN) is 0, not the +1 clamp() would give")


def test_bad_parameters_fall_back():
    node = make_node({"expo": 1.5, "deadzone": 1.0, "max_speed": -0.1, "max_turn": math.nan})
    check(node.settings == {**tt.DEFAULTS, "min_output": tt.DEFAULTS["min_output"]}, f"got {node.settings}")
    check(sum(level == "error" for level, _ in node.logger.lines) == 4, "each bad parameter is logged")
    node = make_node({"deadzone": 0, "expo": True})
    check(node.settings["deadzone"] == 0.0 and node.settings["expo"] == tt.DEFAULTS["expo"],
          "an int 0 is a valid fraction; a bool is not a number")
    node = make_node({"max_speed": 0.4})
    joy(node)
    node.command()
    joy(node, left_y=1.0, lb=1)
    check(close(node.command()[0], 0.4 * 15.0), "max_speed caps forward speed")


def test_step_publishes_stamped_command_and_enabled():
    node = armed_node()
    joy(node, left_y=1.0, lb=1)
    node.step()
    twist = node.publishers["diffbot_base_controller/cmd_vel"].sent[-1]
    enabled = node.publishers["tank_teleop/enabled"].sent[-1]
    check(twist.header.stamp == "stamp" and close(twist.twist.linear.x, 15.0), "stamped full-speed command")
    check(enabled.data is True, "enabled is published")
    joy(node)
    node.step()
    check(node.publishers["tank_teleop/enabled"].sent[-1].data is False, "LB released: disabled published")


# --- wiring and decoupling ---------------------------------------------------------------------------
LAUNCH = PACKAGE / "rocko_env" / "bringup" / "launch"


def test_wheel_separation_matches_the_yaml():
    yaml = (PACKAGE / "rocko_env/bringup/config/rocko_tank_mode_controllers.yaml").read_text()
    separation = float(re.search(r"^\s*wheel_separation: ([0-9.]+)", yaml, re.M).group(1))
    check(separation == tt.WHEEL_SEPARATION_M, f"TankTeleop {tt.WHEEL_SEPARATION_M} != YAML {separation}")
    check(close(tt.FULL_TURN_RAD_S, 93.75), "a full turn alone is 93.75 rad/s")


def test_tank_launch_starts_the_usb_teleop_and_balancing_does_not():
    tank = (LAUNCH / "rocko_tank_mode.launch.py").read_text()
    check('executable="game_controller_node"' in tank and 'executable="TankTeleop.py"' in tank,
          "tank mode starts game_controller_node and TankTeleop.py")
    check("Joystick.py" not in tank, "tank mode does not start balancing's Joystick.py")
    check("gamepad.JOY_AUTOREPEAT_RATE_HZ" in tank, "the joy node's rate comes from gamepad.py")
    check(tank.count('condition=IfCondition(LaunchConfiguration("teleop"))') == 2,
          "teleop:=false must leave out both controller nodes (the bench test publishes cmd_vel itself)")
    for name in ("rocko.launch.py", "rocko_sim.launch.py"):
        text = (LAUNCH / name).read_text()
        check("TankTeleop" not in text and "game_controller_node" not in text, f"{name} must not start the tank teleop")
    for path in ("controllers/joystick/Joystick.py", "controllers/diffdrive/DiffDriveController.py"):
        check("tank_teleop" not in (PACKAGE / "rocko_env" / path).read_text(), f"{path} must not use the tank node")


# --- runner ------------------------------------------------------------------------------------------
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
