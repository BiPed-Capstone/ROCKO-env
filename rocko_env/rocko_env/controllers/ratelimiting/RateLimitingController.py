#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from control_msgs.msg import MultiDOFCommand
from sensor_msgs.msg import JointState
from RateLimiter import RateLimiter

class RateLimitingController(Node):

    def __init__(self):
        super().__init__('ratelimiting_controller')
        # Set up publisher to command controller

        # TODO: publish to MultiDofCommand or plain vel?
        self.left_controller_topic = self.create_publisher(MultiDofCommand, 'left_balancing_pid_controller/reference', 10)
        self.right_controller_topic = self.create_publisher(MultiDofCommand, 'right_balancing_pid_controller/reference', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.command_controller)

        # Set up subscriber to get velocity data
        self.vel_updated_topic = self.create_subscription(
            JointState,
            'joint_states',
            self.vel_updated,
            10)

        # Set up variables for holding data
        self.left_vel = 0
        self.right_vel = 0
        self.left_v0 = 0
        self.left_v1 = 0
        self.right_v0 = 0
        self.right_v1 = 0
        self.init = False # error-proofing for not importing an automatic 0

        # Configure RateLimiter
        # TODO: all of these are completely arbitrary rn. use foxglove to moderate
        # 1st derivative limits should represent the "acceleration limit"
        # 2nd derivative limits should represent the limit for "jerk" (del a/ del t)
        # min/max values should be whatever our intended min and max values are
        self.left_limiter = RateLimiter(
            min_value=-10, max_value=10,
            min_first_derivative_neg=-5, max_first_derivative_pos=5,
            min_second_derivative=-20, max_second_derivative=20            
        )
        self.right_limiter = RateLimiter(
            min_value=-10, max_value=10,
            min_first_derivative_neg=-5, max_first_derivative_pos=5,
            min_second_derivative=-20, max_second_derivative=20
        )

    def command_controller(self):
        if not self.initialized:
            return
        
        # apply ratelimit
        smooth_left_vel = self.left_limiter.limit(self.left_vel, self.left_v0, self.left_v1, self.timer_period)
        smooth_right_vel = self.right_limiter.limit(self.right_vel, self.right_v0, self.right_v1, self.timer_period)

        # back-propogate
        self.left_v1, self.left_v0 = self.left_v0, smooth_left_vel
        self.right_v1, self.right_v0 = self.right_v0, smooth_right_vel

        # publish msgs
        smoothed_velocity_right = MultiDOFCommand()
        smoothed_velocity_right.dof_names = ["left_wheel_joint"]
        smoothed_velocity_right.values = [smooth_right_vel]
        self.left_controller_topic.publish(smoothed_velocity_right)

        smoothed_velocity_left = MultiDOFCommand()
        smoothed_velocity_left.dof_names = ["left_wheel_joint"]
        smoothed_velocity_left.values = [smooth_left_vel]    
        self.left_controller_topic.publish(smoothed_velocity_left)

    def vel_updated(self, msg: JointState):
        # Store current velocity
        self.left_vel = msg.velocity[msg.name.index('left_wheel_joint')]
        self.right_vel = msg.velocity[msg.name.index('right_wheel_joint')]
        self.init = True

def main(args=None):
    rclpy.init(args=args)

    ratelimiting_controller = RateLimitingController()

    rclpy.spin(ratelimiting_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ratelimiting_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()