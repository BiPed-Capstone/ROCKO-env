#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import MultiDOFCommand
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float64
import math
import time

# Teleop watchdog timeout (finding C-3) [s]. PROPOSED value; a human confirms it on the robot.
# 0.2 s = 20 control cycles of 10 ms = 4 x the 0.05 s at which a teleop sender must resend its
# command even while the stick is held still, or a steady stick looks like a dropped link. (The
# laptop gamepad script that did so was removed on 2026-11-22; balancing mode has no sender now.) Overridable with the command_timeout_s
# parameter; this one literal is the default and the fallback for a bad value.
DEFAULT_COMMAND_TIMEOUT_S = 0.2

# Command ramp (finding H-13) [m/s^2]: the most the drive command may change per second.
# An upper bound: the ramp moves one step per timer cycle, so if this node runs late the command
# changes more slowly, never faster.
# Why it exists: a balancing robot cannot change speed at once. To speed up it must first lean, and
# a command that jumps asks the wheels to run out from under the body. In the simulation a bare
# 0.5 m/s step leaned the model 21 deg and a 1.0 m/s step toppled it
# (docs/journal/2026-11-16-teleop-dashboard-logging.md).
# PROPOSED value, chosen in the simulation (a model with estimated parameters), not on the robot;
# a human confirms it. Overridable with the max_linear_accel_m_s2 parameter.
# Only the forward command is ramped. The turn command is NOT: turning speeds one wheel up and the
# other down, which is ASSUMED not to tip the body the way a forward step does. Not checked: the
# simulation is planar, so no tier could test a turn ramp.
DEFAULT_MAX_LINEAR_ACCEL_M_S2 = 0.35


def step_toward(current, target, max_step):
    """Return `current` moved toward `target` by at most `max_step` (>= 0). Same unit for all three."""
    return current + max(-max_step, min(target - current, max_step))


class CommandWatchdog:
    """Remembers when the newest teleop command ARRIVED and says whether it is too old.

    now() must be a monotonic clock in seconds (time.monotonic); it is a parameter only
    so a test can pass a fake clock. Age is measured from update(), which is called when
    a command arrives, never from when the controller last published something.
    Before the first command ever arrives, expired() is True: nobody is driving, and the
    reference is zero anyway.
    """

    def __init__(self, timeout_s, now=time.monotonic):
        self.timeout_s = timeout_s
        self.now = now
        self.last_command_time_s = None

    def update(self):
        self.last_command_time_s = self.now()

    def has_received(self):
        """True once any command has arrived (so "expired" means the link went quiet)."""
        return self.last_command_time_s is not None

    def expired(self):
        if self.last_command_time_s is None:
            return True
        age_s = self.now() - self.last_command_time_s
        # Written as "not (age <= timeout)" so a NaN timeout counts as expired, not as never.
        return not (age_s <= self.timeout_s)


class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')
        # Set up publishers to command pid controllers
        self.velocity_controller_topic = self.create_publisher(MultiDOFCommand, 'velocity_pid_controller/reference', 10)
        # Publishers for feedforward
        self.left_feedforward_topic = self.create_publisher(Float64, 'left_feedforward', 10)
        self.right_feedforward_topic = self.create_publisher(Float64, 'right_feedforward', 10)
        self.timer_period = 0.01  # seconds

        # Teleop watchdog (finding C-3). If the newest gamepad command is older than this,
        # the velocity reference and the turn/feedforward terms are set to zero.
        # Seconds; default DEFAULT_COMMAND_TIMEOUT_S (see its comment). Read once at startup.
        self.declare_parameter('command_timeout_s', DEFAULT_COMMAND_TIMEOUT_S)
        command_timeout_s = self.get_parameter('command_timeout_s').value
        if not (0.0 < command_timeout_s < math.inf):  # also rejects NaN
            self.get_logger().error(
                f"command_timeout_s={command_timeout_s} is not a positive number; "
                f"using {DEFAULT_COMMAND_TIMEOUT_S} s")
            command_timeout_s = DEFAULT_COMMAND_TIMEOUT_S
        self.command_watchdog = CommandWatchdog(command_timeout_s)
        self.command_timed_out = None  # None until the first cycle reports the state
        # transient_local: a subscriber that starts late still receives the current state.
        self.command_timed_out_topic = self.create_publisher(
            Bool, '~/command_timed_out',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Command ramp (finding H-13); see DEFAULT_MAX_LINEAR_ACCEL_M_S2. Read once at startup.
        self.declare_parameter('max_linear_accel_m_s2', DEFAULT_MAX_LINEAR_ACCEL_M_S2)
        self.max_linear_accel_m_s2 = self.get_parameter('max_linear_accel_m_s2').value
        if not (0.0 < self.max_linear_accel_m_s2 < math.inf):  # also rejects NaN
            # <= 0 or NaN would freeze or reverse the ramp; inf would switch it off.
            self.get_logger().error(
                f"max_linear_accel_m_s2={self.max_linear_accel_m_s2} is not a positive number; "
                f"using {DEFAULT_MAX_LINEAR_ACCEL_M_S2} m/s^2")
            self.max_linear_accel_m_s2 = DEFAULT_MAX_LINEAR_ACCEL_M_S2
        # The forward speed sent last cycle [m/s]; the ramp starts from here.
        self.ramped_linear_vel = 0.0

        self.timer = self.create_timer(self.timer_period, self.command_controller)
        
        # Set up subscriber to get robot body vector
        self.robot_body_vector_updated_topic = self.create_subscription(
            TwistStamped,
            'diffbot_base_controller/cmd_vel',
            self.robot_body_vector_updated,
            10)
        
        # Set up variables to hold data
        self.desired_robot_body_vector = TwistStamped()
        
        # Robot character numbers
        self.wheel_radius_meters = 0.06
        
        self.init = False # error-proofing for not importing an automatic 0
        self.wheel_distance = 0.356 # meters
        
    def update_timed_out_state(self, timed_out):
        """Log and publish only when the state changes, not every 10 ms cycle."""
        if timed_out == self.command_timed_out:
            return
        self.command_timed_out = timed_out
        if timed_out and not self.command_watchdog.has_received():
            # Start-up: nobody has driven yet. Not a fault, so not the time-out warning (which
            # would also make a check for "the watchdog fired" pass at every start-up).
            self.get_logger().info("no teleop command yet: the velocity reference is zero")
        elif timed_out:
            self.get_logger().warning(
                "teleop command timed out: zeroing the velocity reference, balance loop keeps running")
        else:
            self.get_logger().info("teleop commands resumed")
        self.command_timed_out_topic.publish(Bool(data=timed_out))

    def command_controller(self):
        # Calculate velocities from body vector
        #
        # COMMS LOSS (C-3): the safe response is to command ZERO VELOCITY, and nothing else.
        # While the robot is upright, NEVER stop publishing, deactivate a controller, or zero a
        # motor output, here or downstream. This robot is an inverted pendulum: it stays up only
        # because the balance loop keeps driving the wheels under it. "Stop the motors" is
        # falling over. A zero velocity reference makes the cascade hold the robot still and
        # upright.
        # Contrast with H-4, the tipover cutoff (AddFeedforwardController / TipoverLatch.hpp):
        # that is the ONE place the motor outputs are zeroed, and only AFTER the robot has
        # already fallen past the tipover angle, when driving the wheels can no longer help.
        self.update_timed_out_state(self.command_watchdog.expired())
        if self.command_timed_out:
            # A timeout is a STEP to zero, on purpose NOT ramped: the watchdog (C-3) promises a
            # zero reference within its timeout, and check C3 holds it to that. Whether it should
            # ramp instead is a human's decision (it changes a safety behavior). Driving then
            # resumes from zero.
            linear_vel = 0.0
            angular_vel = 0.0
        else:
            # Ramp (finding H-13): each cycle the forward command moves at most
            # max_linear_accel_m_s2 * timer_period (10 ms) toward what the operator asked. The turn command is not
            # ramped (see DEFAULT_MAX_LINEAR_ACCEL_M_S2).
            linear_vel = step_toward(
                self.ramped_linear_vel, self.desired_robot_body_vector.twist.linear.x,
                self.max_linear_accel_m_s2 * self.timer_period)
            angular_vel = self.desired_robot_body_vector.twist.angular.z
        self.ramped_linear_vel = linear_vel
        left_vel = linear_vel - angular_vel
        right_vel = linear_vel + angular_vel
        
        # Send setpoints to velocity PIDS
        velocity_msg = MultiDOFCommand()
        velocity_msg.dof_names = ["left_wheel_joint", "right_wheel_joint"]
        velocity_msg.values = [linear_vel, linear_vel]
        self.velocity_controller_topic.publish(velocity_msg)
        
        # Send velocities to pitch PID feedforwards
        left_rad_sec = left_vel / self.wheel_radius_meters
        right_rad_sec = right_vel / self.wheel_radius_meters
        
        left_feedforward_msg = Float64()
        left_feedforward_msg.data = left_rad_sec
        self.left_feedforward_topic.publish(left_feedforward_msg)
        
        right_feedforward_msg = Float64()
        right_feedforward_msg.data = right_rad_sec
        self.right_feedforward_topic.publish(right_feedforward_msg)
        
    def robot_body_vector_updated(self, msg):
        # Store desired robot body vector
        # No per-message logging: a gamepad sends 40-200 messages/s and the INFO lines flooded the log.
        # Keep the whole TwistStamped: command_controller reads .twist.linear.x from it.
        # (This line used to store msg.twist, which has no .twist attribute, so the first
        # command would have raised AttributeError inside the timer.)
        # A command that is not a finite number is dropped, and does NOT count as an arrival, so
        # the watchdog treats its sender as silent. (min() and max() in step_toward do not
        # propagate NaN: a NaN target would ramp the command backwards without limit.)
        if not (math.isfinite(msg.twist.linear.x) and math.isfinite(msg.twist.angular.z)):
            self.get_logger().warning(
                "ignored a drive command that is not a finite number", throttle_duration_sec=1.0)
            return
        self.desired_robot_body_vector = msg
        self.command_watchdog.update()  # time of arrival, from a monotonic clock
        self.init = True

def main(args=None):
    rclpy.init(args=args)

    diff_drive_controller = DiffDriveController()

    rclpy.spin(diff_drive_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    diff_drive_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()