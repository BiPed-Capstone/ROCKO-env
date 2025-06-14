#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import MultiDOFCommand
from rocko_env.controllers.utils.RateLimiter import RateLimiter
from std_msgs.msg import Float64
import math

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')
        # Set up publishers to command pid controllers
        self.velocity_controller_topic = self.create_publisher(MultiDOFCommand, 'velocity_pid_controller/reference', 10)
        # Publishers for feedforward
        self.left_feedforward_topic = self.create_publisher(Float64, 'left_feedforward', 10)
        self.right_feedforward_topic = self.create_publisher(Float64, 'right_feedforward', 10)
        self.timer_period = 0.01  # seconds
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
        
        # Configure RateLimiter
        # TODO: all of these are completely arbitrary rn. use foxglove to moderate
        # 1st derivative limits should represent the "acceleration limit"
        # 2nd derivative limits should represent the limit for "jerk" (del a/ del t)
        # min/max values should be whatever our intended min and max values are
        self.left_limiter = RateLimiter(
            min_value=-1000, max_value=1000,
            min_first_derivative_neg=-150, max_first_derivative_pos=150,
            min_second_derivative=-150, max_second_derivative=150            
        )
        self.right_limiter = RateLimiter(
            min_value=-1000, max_value=1000,
            min_first_derivative_neg=-150, max_first_derivative_pos=150,
            min_second_derivative=-150, max_second_derivative=150
        )
        
        # Set up variables for holding data
        self.left_vel = 0
        self.right_vel = 0
        self.left_v0 = 0
        self.left_v1 = 0
        self.right_v0 = 0
        self.right_v1 = 0
        self.init = False # error-proofing for not importing an automatic 0
        self.wheel_distance = 0.356 # meters
        
    def command_controller(self):
        # self.desired_robot_body_vector.linear.x = 1
        
        # apply ratelimit
        smooth_left_vel = self.left_limiter.limit(self.left_vel, self.left_v0, self.left_v1, self.timer_period)
        smooth_right_vel = self.right_limiter.limit(self.right_vel, self.right_v0, self.right_v1, self.timer_period)

        #  smooth_left_vel = self.left_vel
        #  smooth_right_vel = self.right_vel   

        # back-propogate
        self.left_v1, self.left_v0 = self.left_v0, smooth_left_vel
        self.right_v1, self.right_v0 = self.right_v0, smooth_right_vel
         
        # Calculate velocities from body vector
        linear_vel = self.desired_robot_body_vector.twist.linear.x
        angular_vel = self.desired_robot_body_vector.twist.angular.z
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
        self.get_logger().info("Callback triggered")
        self.get_logger().info(f"Received Twist: {msg.twist}")
        self.desired_robot_body_vector = msg.twist
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