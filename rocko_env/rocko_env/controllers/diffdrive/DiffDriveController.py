#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import MultiDOFCommand
from std_msgs.msg import Float64

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')
        # Set up publishers to command pid controllers
        self.left_controller_topic = self.create_publisher(MultiDOFCommand, 'left_velocity_pid_controller/reference', 10)
        self.right_controller_topic = self.create_publisher(MultiDOFCommand, 'right_velocity_pid_controller/reference', 10)
        # Publishers for feedforward
        self.left_feedforward_topic = self.create_publisher(Float64, 'left_feedforward', 10)
        self.right_feedforward_topic = self.create_publisher(Float64, 'right_feedforward', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.command_controller)
        
        # Set up subscriber to get robot body vector
        self.robot_body_vector_updated_topic = self.create_subscription(
            MultiDOFCommand,
            'robot_body_vector',
            self.robot_body_vector_updated,
            10)
        
        # Set up variables to hold data
        self.desired_robot_body_vector = Twist()
        
        # Robot character numbers
        self.wheel_radius_meters = 0.072
        
        
    def command_controller(self):
        self.desired_robot_body_vector.linear.x = 1
        
        # Calculate velocities from body vector
        linear_vel = self.desired_robot_body_vector.linear.x
        angular_vel = self.desired_robot_body_vector.angular.z
        left_vel = linear_vel - angular_vel
        right_vel = linear_vel + angular_vel
        
        # Send setpoints to velocity PIDS
        left_velocity_msg = MultiDOFCommand()
        left_velocity_msg.dof_names = ["left_wheel_joint"]
        left_velocity_msg.values = [left_vel]
        self.left_controller_topic.publish(left_velocity_msg)
        
        right_velocity_msg = MultiDOFCommand()
        right_velocity_msg.dof_names = ["right_wheel_joint"]
        right_velocity_msg.values = [right_vel]
        self.right_controller_topic.publish(right_velocity_msg)
        
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
        self.desired_robot_body_vector = msg.twist

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