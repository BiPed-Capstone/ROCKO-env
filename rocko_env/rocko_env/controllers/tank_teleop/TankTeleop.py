#!/usr/bin/env python3
"""Tank mode's teleop: a game controller plugged into the Pi's USB drives the robot. Tank mode only;
balancing mode has its own nodes and never starts this one.

    controller --USB--> joy's game_controller_node --/joy--> THIS NODE --cmd_vel--> diff_drive_controller

SPLIT ARCADE: the LEFT stick, up and down, drives forward and back; the RIGHT stick, left and right,
turns. Hold the LEFT bumper (LB) to drive: let go and the command is zero.

Every PUBLISH_PERIOD_S this node publishes one command on diffbot_base_controller/cmd_vel and whether
driving is enabled on tank_teleop/enabled (std_msgs/Bool; the dashboard shows it). The command is
zero unless all of these hold:
  * a /joy message arrived within gamepad.JOY_TIMEOUT_S (the joy node republishes 20 times a second
    while a controller is connected and goes SILENT when it is unplugged);
  * LB is held;
  * LB has been seen released since the controller was last silent or since start-up
    (gamepad.DeadmanSwitch explains why).
If this node itself dies, diff_drive_controller stops the wheels after its cmd_vel_timeout (1.0 s,
rocko_tank_mode_controllers.yaml).

From stick to wheel: gamepad.shape() (deadzone, expo) on each stick, gamepad.mix() so no wheel is
asked for more than full speed, then the full-speed scales below. Guide:
docs/guides/drive-with-usb-controller.md.
"""
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import TwistStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

from rocko_env.teleop import gamepad

# FULL-RANGE SCALES: what a full command (1.0) means. A full stick forward is 100 % motor duty, and a
# full turn alone spins each wheel at that same speed (gamepad.mix() needs the two to match).
# Arithmetic, not measured; raising tank mode's top speed to this is proposal 0014 (owner to confirm):
#   Motor12VoltQuadEncoder::write (Motor12VoltQuadEncoder.cpp:231, MAX_RAD_PER_SEC and
#   kStictionDutyPercent in its .hpp): duty % = wheel rad/s / 119.4 * 100 + 2, clamped to 100,
#   so 100 % at 117 rad/s = 14.9 m/s at the 0.127 m wheel radius; 15.0 is that rounded up.
#   diff_drive_controller: wheel speed = linear.x +/- angular.z * wheel_separation / 2, so a wheel at
#   15.0 m/s from turning alone needs angular.z = 15.0 / (0.32 / 2) = 93.75 rad/s.
FULL_WHEEL_SPEED_M_S = 15.0
WHEEL_SEPARATION_M = 0.32  # MUST match wheel_separation in rocko_tank_mode_controllers.yaml
#                            (tools/tank_teleop_test.py fails if the two differ). A copy, not read from
#                            the controller, because that value belongs to another node's parameters and
#                            asking for it at start-up would make this node wait on diff_drive_controller.
FULL_TURN_RAD_S = FULL_WHEEL_SPEED_M_S / (WHEEL_SEPARATION_M / 2)  # [rad/s] 93.75; + = left (CCW)

DEADMAN_BUTTON = gamepad.BUTTON_LEFT_BUMPER  # hold LB to drive
PUBLISH_PERIOD_S = 0.02  # [s] 50 Hz: a released LB becomes a zero command within 20 ms of the joy node's
#                          message. The wheels then slow at diff_drive_controller's acceleration limit
#                          (linear.x.max_deceleration in the tank YAML, proposal 0015: about 0.5 s from
#                          full speed).

# The tunables a student would want to try, each a fraction in [0, 1]. They are ROS parameters of this
# node. max_speed and max_turn are also launch arguments (pi/launch.sh tank max_speed:=0.4); to
# change another, add it to the tank_teleop node's `parameters` in rocko_tank_mode.launch.py.
DEFAULTS = {
    "deadzone": 0.10,    # stick travel that counts as zero; must stay below 1
    "expo": 0.6,         # 0 = straight line, 1 = pure cube; 0.6 = gentle near the centre
    "min_output": 0.0,   # command just outside the deadzone (0.2 skips most of the motors' hum band)
    "max_speed": 1.0,    # cap on any wheel, fraction of FULL_WHEEL_SPEED_M_S
    "max_turn": 1.0,     # turn stick strength
}


class TankTeleop(Node):
    def __init__(self):
        super().__init__("tank_teleop")
        self.settings = {name: self.fraction_parameter(name, default) for name, default in DEFAULTS.items()}
        self.now = time.monotonic   # [s] monotonic clock; a parameter so the test can fake time
        self.axes = []              # the newest /joy message's axes and buttons
        self.buttons = []
        self.last_joy_s = None      # [s] when the newest /joy arrived (None: never)
        self.deadman = gamepad.DeadmanSwitch()
        self.was_enabled = False
        self.was_heard = False
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.cmd_publisher = self.create_publisher(TwistStamped, "diffbot_base_controller/cmd_vel", 10)
        self.enabled_publisher = self.create_publisher(Bool, "tank_teleop/enabled", 10)
        self.create_timer(PUBLISH_PERIOD_S, self.step)
        self.get_logger().info(f"tank teleop: hold LB to drive; settings {self.settings}")

    def fraction_parameter(self, name, default):
        """A [0, 1] parameter; anything else (and a deadzone of 1, which leaves no stick) falls back
        to the default with an error."""
        # dynamic_typing: an int typed by hand (deadzone:=0) must reach the check below, not crash the
        # node with a type error before it.
        value = self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True)).value
        is_number = isinstance(value, (int, float)) and not isinstance(value, bool)
        if not (is_number and 0.0 <= value and (value < 1.0 if name == "deadzone" else value <= 1.0)):  # NaN fails too
            self.get_logger().error(f"{name}={value} is not a fraction in [0, 1]; using {default}")
            return default
        return float(value)

    def on_joy(self, msg):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.last_joy_s = self.now()

    def command(self):
        """(linear.x [m/s], angular.z [rad/s, + = left], enabled) from the newest /joy and its age."""
        heard = self.last_joy_s is not None and self.now() - self.last_joy_s <= gamepad.JOY_TIMEOUT_S
        # A message too short to hold the deadman button or the two sticks (another node's layout,
        # e.g. joy_node instead of game_controller_node) counts as "not held": it must not drive.
        complete = len(self.buttons) > DEADMAN_BUTTON and len(self.axes) > gamepad.AXIS_RIGHT_X
        held = complete and self.buttons[DEADMAN_BUTTON] == 1  # (DeadmanSwitch ignores it when not heard)
        enabled = self.deadman.update(heard, held)
        self.log_changes(heard, enabled)
        if not enabled:
            return 0.0, 0.0, False
        s = self.settings
        # No sign flips: the joy node's convention (up = +1, left = +1) is the robot's (gamepad.py).
        forward = gamepad.shape(self.axes[gamepad.AXIS_LEFT_Y], s["deadzone"], s["expo"], s["min_output"])
        turn_left = gamepad.shape(self.axes[gamepad.AXIS_RIGHT_X], s["deadzone"], s["expo"], s["min_output"])
        forward, turn_left = gamepad.mix(forward, turn_left, s["max_speed"], s["max_turn"])
        return forward * FULL_WHEEL_SPEED_M_S, turn_left * FULL_TURN_RAD_S, True

    def log_changes(self, heard, enabled):
        """One log line per change (not per cycle), so the launch output says what happened."""
        if heard != self.was_heard:
            self.get_logger().info("controller heard on /joy" if heard else
                                   f"no /joy for {gamepad.JOY_TIMEOUT_S} s: controller gone, stopping")
            self.was_heard = heard
        if enabled != self.was_enabled:
            self.get_logger().info("DRIVE ENABLED (LB held)" if enabled else "drive disabled")
            self.was_enabled = enabled

    def step(self):
        linear, angular, enabled = self.command()
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()  # diff_drive_controller ages commands by this
        twist.twist.linear.x = linear
        twist.twist.angular.z = angular
        self.cmd_publisher.publish(twist)
        self.enabled_publisher.publish(Bool(data=enabled))


def main(args=None):
    rclpy.init(args=args)
    node = TankTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
