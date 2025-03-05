#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from control_msgs.msg import MultiDOFCommand
from rocko_interfaces.msg import Icm20948Data

class DiffDriveController(Node):

    def __init__(self):
        super().__init__('diff_drive_controller')
        # Set up publishers to command controller
        self.left_controller_topic = self.create_publisher(MultiDOFCommand, 'left_velocity_pid_controller/reference', 10)
        self.right_controller_topic = self.create_publisher(MultiDOFCommand, 'right_velocity_pid_controller/reference', 10)
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
        
        # Set up variables to hold robot parameters
        self.wheel_separation = 0.4368
        self.wheel_radius = 0.072

    def command_controller(self):
        # Calculate velocities from body vector
        linear_vel = self.desired_robot_body_vector.linear.x
        angular_vel = self.desired_robot_body_vector.angular.z
        left_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
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