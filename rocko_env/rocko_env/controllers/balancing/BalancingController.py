#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from rocko_interfaces.msg import Icm20948Data

class BalancingController(Node):

    def __init__(self):
        super().__init__('balancing_controller')
        # Set up publisher to command controller
        self.command_controller_topic = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.command_controller)
        
        # Set up subscriber to get velocity data
        self.vel_updated_topic = self.create_subscription(
            JointState,
            'joint_states',
            self.vel_updated,
            10)
        
        # Set up subscriber to get IMU data
        self.imu_updated_topic = self.create_subscription(
            Icm20948Data,
            'icm20948_data',
            self.imu_updated,
            10)
        
        # Set up subscriber to get robot body vector
        self.robot_body_vector_updated_topic = self.create_subscription(
            TwistStamped,
            'robot_body_vector',
            self.robot_body_vector_updated,
            10)
        
        # Set up variables to hold data
        self.left_vel = 0
        self.right_vel = 0
        self.current_angles = Icm20948Data()
        self.desired_robot_body_vector = Twist()

    def command_controller(self):
        msg = TwistStamped()
        # Do calcs here for equations of motion
        
        # Set values of vectors based on equations of motion results
        linearVector = Vector3()
        linearVector.x = self.desired_robot_body_vector.linear.x
        linearVector.y = self.desired_robot_body_vector.linear.y
        linearVector.z = self.desired_robot_body_vector.linear.z
        
        angularVector = Vector3()
        angularVector.x = self.desired_robot_body_vector.angular.x
        angularVector.y = self.desired_robot_body_vector.angular.y
        angularVector.z = self.desired_robot_body_vector.angular.z

        twist = Twist()
        twist.linear = linearVector
        twist.angular = angularVector
        # publish the desired valocity for the robot
        msg.twist = twist
        msg.header.stamp = self.get_clock().now().to_msg()
        self.command_controller_topic.publish(msg)
        
    def vel_updated(self, msg: JointState):
        # Store current velocity
        self.left_vel = msg.velocity[msg.name.index('left_wheel_joint')]
        self.right_vel = msg.velocity[msg.name.index('right_wheel_joint')]
        
    def imu_updated(self, msg):
        # Store current imu data
        self.current_angles.yaw = msg.yaw
        self.current_angles.roll = msg.roll
        self.current_angles.pitch = msg.pitch
        
    def robot_body_vector_updated(self, msg):
        # Store desired robot body vector
        self.desired_robot_body_vector = msg.twist


def main(args=None):
    rclpy.init(args=args)

    balancing_controller = BalancingController()

    rclpy.spin(balancing_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    balancing_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()