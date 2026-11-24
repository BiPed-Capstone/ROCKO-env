#!/usr/bin/env python3
"""Host test for balancing mode's USB-controller teleop (BalanceTeleop.py). No ROS, no robot.

Run from the repo root:  python3 -I tools/balance_teleop_test.py

Reuses the stand-ins of tools/tank_teleop_test.py (fake rclpy, messages, clock) and runs the SAME gate
checks against BalanceTeleop: zero with no /rocko/state, a stale one, a disabled one or another mode's; zero
until the sticks have been centred since the last enable; zero after JOY_TIMEOUT_S of silence; NaN is zero.
Then the parts that are balancing's own: the full-scale numbers (the old Joystick.py's 10 m/s and 35 rad/s),
the default caps (0.05 forward, 0.2 turn), the topic DiffDriveController.py subscribes to, and that it shares nothing with tank mode.
What it cannot show: the joy node, ROS timing, DiffDriveController, the balance loop, the robot. Not a tier.
"""
import math
import pathlib
import re
import sys

TOOLS = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))  # python -I leaves the script's folder off the path
import tank_teleop_test as base  # noqa: E402
from tank_teleop_test import PACKAGE, check, close, gamepad, joy, state, armed_node, make_node  # noqa: E402

bt = base.load_teleop("balance_teleop", "BalanceTeleop.py")


def make_balance(params=None):
    return make_node(bt, "BalanceTeleop", params)


def test_balance_gate():
    base.gate_checks(make_balance, "balance", "tank", 10.0, 35.0)


def test_balance_defaults_and_bad_parameters():
    check(bt.DEFAULTS["max_speed"] == 0.05 and bt.DEFAULTS["max_turn"] == 0.2, "caps default to 0.05 / 0.2")
    base.gate_parameter_checks(make_balance, bt.DEFAULTS, bt)


def test_full_scale_and_caps():
    node = armed_node(make_balance, "balance")
    joy(node, left_y=1.0)
    linear, angular, sending = node.command()
    check(sending and close(linear, 10.0 * 0.05) and angular == 0.0, f"full stick at the default cap = 0.5 m/s, got {linear}")
    joy(node, right_x=1.0)
    linear, angular, _ = node.command()
    # gamepad.mix() applies max_turn to the stick and then max_speed to both: 0.2 * 0.05 of full turn.
    check(linear == 0.0 and close(angular, 35.0 * 0.2 * 0.05), f"full left turn at the default caps, got {angular}")
    node = armed_node(lambda: make_balance({"max_speed": 1.0, "max_turn": 1.0}), "balance")
    joy(node, left_y=1.0)
    check(close(node.command()[0], 10.0), "max_speed 1.0: full stick = 10 m/s (Joystick.py's linear_scale)")
    joy(node, right_x=1.0)
    check(close(node.command()[1], 35.0), "max_turn 1.0: full left = +35 rad/s (Joystick.py's angular_scale)")
    joy(node, left_y=1.0, right_x=1.0)
    linear, angular, _ = node.command()
    check(close(linear / 10.0 + angular / 35.0, 1.0), "both sticks over share the budget (gamepad.mix)")


def test_the_numbers_come_from_the_old_joystick_node():
    text = (PACKAGE / "rocko_env/controllers/joystick/Joystick.py").read_text()
    linear = float(re.search(r"self\.linear_scale = (-?[0-9.]+)", text).group(1))
    angular = float(re.search(r"self\.angular_scale = (-?[0-9.]+)", text).group(1))
    check(abs(linear) == bt.FULL_LINEAR_M_S and abs(angular) == bt.FULL_TURN_RAD_S,
          f"Joystick.py scales {linear}, {angular} != {bt.FULL_LINEAR_M_S}, {bt.FULL_TURN_RAD_S}")


def test_publishes_on_the_topic_diffdrive_listens_to():
    diff = (PACKAGE / "rocko_env/controllers/diffdrive/DiffDriveController.py").read_text()
    check("'diffbot_base_controller/cmd_vel'" in diff, "DiffDriveController subscribes to diffbot_base_controller/cmd_vel")
    node = armed_node(make_balance, "balance")
    joy(node, left_y=1.0)
    node.step()
    twist = node.publishers["diffbot_base_controller/cmd_vel"].sent[-1]
    check(twist.header.stamp == "stamp" and close(twist.twist.linear.x, 0.5), "stamped command at the default cap (0.5 m/s)")
    check(node.publishers["balance_teleop/enabled"].sent[-1].data is True, "enabled flag published")
    cmd = node.publishers["diffbot_base_controller/cmd_vel"]
    node.now.t += gamepad.JOY_TIMEOUT_S + 0.1
    state(node, "balance")
    node.step()
    check(cmd.sent[-1].twist.linear.x == 0.0, "enabled, controller silent: zero velocity is sent (C-3)")
    sent_before = len(cmd.sent)
    state(node, "balance", enabled=False)
    node.step()
    check(len(cmd.sent) == sent_before, "disabled: no command sent")


def test_decoupled_from_tank_mode():
    text = (PACKAGE / "rocko_env/controllers/balance_teleop/BalanceTeleop.py").read_text()
    check(not re.search(r"^\s*(import|from)\s.*(tank_teleop|TankTeleop)", text, re.M), "does not import the tank node")
    imports = re.findall(r"^from rocko_env[.\w]* import .*$", text, re.M)
    check(imports == ["from rocko_env.teleop import gamepad"], f"shares only teleop/gamepad.py, imports {imports}")


def main():
    tests = [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as error:  # noqa: BLE001
            failed += 1
            print(f"FAIL  {name}: {error!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
