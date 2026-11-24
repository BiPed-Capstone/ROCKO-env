#!/usr/bin/env python3
"""Host test for tank mode's USB-controller teleop. No ROS, no controller, no robot.

Run from the repo root:  python3 -I tools/tank_teleop_test.py

It checks the shared module rocko_env/teleop/gamepad.py as plain functions, then loads
controllers/tank_teleop/TankTeleop.py with small stand-ins for rclpy and the message types and a fake
clock, and feeds it /joy messages:
  * gamepad: the /joy layout (LB = 9, left Y = 1, right X = 2), the 0.25 s silence timeout; shape():
    deadzone, no jump at its edge, full stick = 1, expo value, symmetry, clamping, min_output; mix(): no
    wheel above max_speed, the turn survives at full forward;
  * TankTeleop (no hold-LB deadman any more; the gate is /rocko/state): zero with no /joy, with no or
    stale /rocko/state (and a throttled warning naming the supervisor), with the state disabled or in
    another mode, until the sticks have been centred since the last enable, after JOY_TIMEOUT_S of
    silence; a NaN or infinite stick is a zero command; full left stick up = 15 m/s forward, right stick
    right = a right turn (negative angular.z), and the two together keep both wheels within full speed;
    bad parameters fall back to the defaults; step() publishes a stamped command and the enabled flag.
    The same gate checks run against BalanceTeleop in tools/balance_teleop_test.py;
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


# --- TankTeleop.py, with stand-ins for ROS ----------------------------------------------------------
class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text, **_):
        self.lines.append(("info", text))

    def error(self, text, **_):
        self.lines.append(("error", text))

    def warning(self, text, **kwargs):
        self.lines.append(("warning", text))
        self.last_warning_kwargs = kwargs


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
        self.subscriptions = {}

    def get_logger(self):
        return self.logger

    def declare_parameter(self, name, default, descriptor=None):
        return SimpleNamespace(value=self.param_overrides.get(name, default))

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = qos
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


def load_teleop(directory, filename):
    """Import a teleop node from controllers/<directory>/<filename> with stand-ins for ROS."""
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod
    module("rclpy", executors=SimpleNamespace(ExternalShutdownException=Exception))
    module("rclpy.executors", ExternalShutdownException=Exception)
    module("rclpy.node", Node=FakeNode)
    module("rclpy.qos", QoSProfile=lambda **kw: kw,
           DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL="transient_local"),
           ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"))
    module("rcl_interfaces")
    module("rcl_interfaces.msg", ParameterDescriptor=lambda **kw: kw)
    module("rocko_interfaces")
    module("rocko_interfaces.msg", RobotState=Msg)
    module("geometry_msgs")
    module("geometry_msgs.msg", TwistStamped=Msg)
    module("sensor_msgs")
    module("sensor_msgs.msg", Joy=Msg)
    module("std_msgs")
    module("std_msgs.msg", Bool=Msg)
    path = PACKAGE / "rocko_env" / "controllers" / directory / filename
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def make_node(mod, cls_name, params=None):
    FakeNode.param_overrides = params or {}
    node = getattr(mod, cls_name)()
    node.now = Clock()
    return node


def joy(node, left_y=0.0, right_x=0.0, n_axes=6):
    axes = [0.0] * 6
    axes[gamepad.AXIS_LEFT_Y], axes[gamepad.AXIS_RIGHT_X] = left_y, right_x
    node.on_joy(Msg(axes=axes[:n_axes], buttons=[0] * 21))


def state(node, mode, enabled=True):
    node.on_state(Msg(mode=mode, enabled=enabled, phase="ready"))


def armed_node(make, mode):
    """A node that is enabled in `mode`, hears the controller, and has seen the sticks centred."""
    node = make()
    state(node, mode)
    joy(node)
    node.command()
    return node


# --- the shared gate: the same checks run against both teleop nodes (see balance_teleop_test.py) -----------
def gate_checks(make, mode, other_mode, full_linear, full_turn):
    """`make()` builds a node; returns nothing, raises AssertionError on failure."""
    check(make().command() == (0.0, 0.0, False), "nothing heard at all: zero")

    node = make()
    joy(node, left_y=1.0)
    check(node.command() == (0.0, 0.0, False), "/joy but no /rocko/state: zero")
    check(any(level == "warning" and "supervisor" in text for level, text in node.logger.lines),
          "no state: a warning names the supervisor")
    check(node.logger.last_warning_kwargs.get("throttle_duration_sec") == 10.0, "the warning is throttled to 10 s")

    node = make()
    state(node, mode)
    check(node.command() == (0.0, 0.0, False), "enabled but no /joy: zero")

    node = armed_node(make, mode)
    joy(node, left_y=1.0)
    linear, angular, sending = node.command()
    check(sending and close(linear, full_linear * node.settings["max_speed"]) and angular == 0.0,
          f"left stick up = forward (+x), got {linear}")
    joy(node, right_x=-1.0)
    linear, angular, _ = node.command()
    check(linear == 0.0 and angular < 0.0, f"right stick right = right turn (-z), got {angular}")
    joy(node, right_x=1.0)
    check(node.command()[1] > 0.0, "right stick left = left turn (+z)")

    node = armed_node(make, mode)
    state(node, mode, enabled=False)
    joy(node, left_y=1.0)
    check(node.command() == (0.0, 0.0, False), "state disabled: zero")

    node = armed_node(make, mode)
    state(node, other_mode)
    joy(node, left_y=1.0)
    check(node.command() == (0.0, 0.0, False), f"state is {other_mode} mode: zero")

    # Stick already over when enable arrives: zero until it has been centred once.
    node = make()
    state(node, mode, enabled=False)
    joy(node, left_y=1.0)
    node.command()
    state(node, mode, enabled=True)
    check(node.command() == (0.0, 0.0, False), "stick held over at enable: still zero")
    joy(node)
    node.command()
    joy(node, left_y=1.0)
    check(node.command()[2] is True, "centred once, then pushed: drives")
    # ... and again after the next disable -> enable.
    state(node, mode, enabled=False)
    node.command()
    state(node, mode, enabled=True)
    check(node.command()[2] is False, "re-enabled with the stick still over: zero again")

    node = armed_node(make, mode)
    joy(node, left_y=1.0)
    check(node.command()[2] is True, "driving")
    node.now.t += gamepad.JOY_TIMEOUT_S - 0.01
    state(node, mode)
    check(node.command()[2] is True, "just inside the /joy timeout: still driving")
    node.now.t += 0.02
    state(node, mode)
    check(node.command() == (0.0, 0.0, False), "past the /joy timeout: zero")
    joy(node, left_y=1.0)  # the joy node after a replug, stick still over
    check(node.command()[2] is False, "a reconnect with the stick over must not drive until centred")
    check(any("controller gone" in text for _, text in node.logger.lines), "the silence is logged")

    node = armed_node(make, mode)
    joy(node, left_y=1.0)
    node.now.t += 3.1   # joy fresh again, but the supervisor's 2 Hz messages stopped
    joy(node, left_y=1.0)
    check(node.command() == (0.0, 0.0, False), "state older than STATE_TIMEOUT_S: zero")

    node = make()
    state(node, mode)
    joy(node, n_axes=2)
    node.command()
    joy(node, left_y=1.0, n_axes=2)
    check(node.command()[2] is False, "a message without a right-stick axis never counts as centred")

    node = armed_node(make, mode)
    joy(node, left_y=math.nan, right_x=math.inf)
    linear, angular, _ = node.command()
    check(linear == 0.0 and angular == 0.0, f"a NaN or infinite stick must give zero, got {linear}, {angular}")
    check(gamepad.shape(math.nan, 0.1, 0.6) == 0.0, "shape(NaN) is 0, not the +1 clamp() would give")

    node = make()
    check("/rocko/state" in node.subscriptions, "subscribes to /rocko/state")
    qos = node.subscriptions["/rocko/state"]
    check(qos == {"depth": 1, "reliability": "reliable", "durability": "transient_local"},
          f"/rocko/state QoS must match the supervisor's, got {qos}")


def gate_parameter_checks(make_with, defaults, mod):
    node = make_with({"expo": 1.5, "deadzone": 1.0, "max_speed": -0.1, "max_turn": math.nan})
    check(node.settings == defaults, f"got {node.settings}")
    check(sum(level == "error" for level, _ in node.logger.lines) == 4, "each bad parameter is logged")
    node = make_with({"deadzone": 0, "expo": True})
    check(node.settings["deadzone"] == 0.0 and node.settings["expo"] == defaults["expo"],
          "an int 0 is a valid fraction; a bool is not a number")


# --- TankTeleop.py ------------------------------------------------------------------------------------------
tt = load_teleop("tank_teleop", "TankTeleop.py")


def make_tank(params=None):
    return make_node(tt, "TankTeleop", params)


def test_tank_gate():
    gate_checks(make_tank, "tank", "balance", 15.0, 93.75)


def test_tank_has_no_deadman_and_ignores_buttons():
    text = (PACKAGE / "rocko_env/controllers/tank_teleop/TankTeleop.py").read_text()
    check("DeadmanSwitch" not in text and "BUTTON_LEFT_BUMPER" not in text, "the LB deadman is gone")
    node = armed_node(make_tank, "tank")
    node.on_joy(Msg(axes=[0.0, 1.0, 0.0, 0, 0, 0], buttons=[0] * 21))  # LB (9) not held
    check(node.command()[2] is True, "drives with no button held")


def test_tank_full_scale_and_forward_plus_turn_keep_both_wheels_within_full():
    node = armed_node(make_tank, "tank")
    joy(node, left_y=1.0)
    check(close(node.command()[0], 15.0), "full stick = 15 m/s")
    joy(node, left_y=1.0, right_x=-1.0)
    linear, angular, _ = node.command()
    half_track = tt.WHEEL_SEPARATION_M / 2
    for wheel in (linear - angular * half_track, linear + angular * half_track):
        check(abs(wheel) <= tt.FULL_WHEEL_SPEED_M_S + 1e-9, f"a wheel was asked for {wheel} m/s")


def test_tank_bad_parameters_fall_back():
    gate_parameter_checks(make_tank, tt.DEFAULTS, tt)
    node = make_tank({"max_speed": 0.4})
    state(node, "tank")
    joy(node)
    node.command()
    joy(node, left_y=1.0)
    check(close(node.command()[0], 0.4 * 15.0), "max_speed caps forward speed")


def test_tank_step_publishes_stamped_command_and_enabled():
    node = armed_node(make_tank, "tank")
    joy(node, left_y=1.0)
    node.step()
    twist = node.publishers["diffbot_base_controller/cmd_vel"].sent[-1]
    enabled = node.publishers["tank_teleop/enabled"].sent[-1]
    check(twist.header.stamp == "stamp" and close(twist.twist.linear.x, 15.0), "stamped full-speed command")
    check(enabled.data is True, "enabled is published")
    cmd = node.publishers["diffbot_base_controller/cmd_vel"]
    node.now.t += gamepad.JOY_TIMEOUT_S + 0.1   # enabled, but the controller went silent: zero IS sent (lost input, C-3)
    state(node, "tank")
    node.step()
    check(cmd.sent[-1].twist.linear.x == 0.0, "enabled, controller silent: a zero command is sent")
    sent_before = len(cmd.sent)
    state(node, "tank", enabled=False)
    node.step()
    check(node.publishers["tank_teleop/enabled"].sent[-1].data is False, "disabled published")
    check(len(cmd.sent) == sent_before, "disabled: no command sent (an inactive diff_drive_controller warns on each)")


# --- wiring and decoupling ---------------------------------------------------------------------------
LAUNCH = PACKAGE / "rocko_env" / "bringup" / "launch"


def test_wheel_separation_matches_the_yaml():
    yaml = (PACKAGE / "rocko_env/bringup/config/rocko_tank_mode_controllers.yaml").read_text()
    separation = float(re.search(r"^\s*wheel_separation: ([0-9.]+)", yaml, re.M).group(1))
    check(separation == tt.WHEEL_SEPARATION_M, f"TankTeleop {tt.WHEEL_SEPARATION_M} != YAML {separation}")
    check(close(tt.FULL_TURN_RAD_S, 93.75), "a full turn alone is 93.75 rad/s")


def test_launch_files_wire_the_teleops_and_the_supervised_flag():
    tank = (LAUNCH / "rocko_tank_mode.launch.py").read_text()
    check('executable="game_controller_node"' in tank and 'executable="TankTeleop.py"' in tank,
          "tank mode starts game_controller_node and TankTeleop.py")
    check("Joystick.py" not in tank, "tank mode does not start balancing's Joystick.py")
    check("gamepad.JOY_AUTOREPEAT_RATE_HZ" in tank, "the joy node's rate comes from gamepad.py")
    check('IfCondition(LaunchConfiguration("teleop"))' in tank
          and 'IfCondition(AndSubstitution(LaunchConfiguration("teleop"), NotSubstitution(supervised)))' in tank,
          "teleop:=false must leave out both controller nodes (the bench test publishes cmd_vel itself); "
          "supervised leaves out only the joy node")
    check(tank.count('"--inactive"') == 1 and 'condition=IfCondition(supervised)' in tank
          and 'condition=UnlessCondition(supervised)' in tank,
          "supervised:=true spawns diffbot_base_controller with --inactive, false as before")
    check('"supervised", default_value="false"' in tank, "supervised defaults to false: a plain launch is unchanged")
    check(re.search(r"GroupAction\(\s*\[\s*foxglove_bridge,\s*dashboard_node\(.*?condition=UnlessCondition\(supervised\)",
                    tank, re.S), "supervised leaves out the dashboard and foxglove (the base launch runs them)")
    balance = (LAUNCH / "rocko.launch.py").read_text()
    check("TankTeleop" not in balance and 'executable="Joystick.py"' not in balance, "rocko.launch.py starts neither TankTeleop nor Joystick.py")
    check('executable="BalanceTeleop.py"' in balance and 'executable="game_controller_node"' in balance,
          "rocko.launch.py starts game_controller_node and BalanceTeleop.py")
    check('"supervised", default_value="false"' in balance and balance.count('"--inactive"') == 1,
          "rocko.launch.py: supervised defaults to false; the cascade spawns --inactive only when supervised")
    check(balance.count('"--activate-as-group"') == 1, "the unsupervised spawn keeps --activate-as-group")
    sim = (LAUNCH / "rocko_sim.launch.py").read_text()
    check("TankTeleop" not in sim and "game_controller_node" not in sim and "BalanceTeleop" not in sim,
          "rocko_sim.launch.py is untouched by the teleop split")
    for path in ("controllers/joystick/Joystick.py", "controllers/diffdrive/DiffDriveController.py",
                 "controllers/balance_teleop/BalanceTeleop.py"):
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
