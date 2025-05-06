#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import Encoder

import numpy as np
import math

from rocko_interfaces.srv import QuadEncoderData
from std_msgs.msg import Float64

class QuadEncoders(Node):

    def __init__(self):
        super().__init__('node')
        # Handle any hardware initialization here
        self.declare_parameter('left_a_pin', 0)
        left_a_pin: int = self.get_parameter('left_a_pin').get_parameter_value().integer_value
        self.declare_parameter('left_b_pin', 0)
        left_b_pin: int = self.get_parameter('left_b_pin').get_parameter_value().integer_value
        self.declare_parameter('right_a_pin', 0)
        right_a_pin: int = self.get_parameter('right_a_pin').get_parameter_value().integer_value
        self.declare_parameter('right_b_pin', 0)
        right_b_pin: int = self.get_parameter('right_b_pin').get_parameter_value().integer_value
                
        self.left_enc = Encoder.Encoder(left_a_pin, left_b_pin)
        self.right_enc = Encoder.Encoder(right_b_pin, right_a_pin) # Right encoder is backwards, so we swap pins in code
        
        self.left_last_position = 0
        self.right_last_position = 0
        self.prev_vels = [0, 0, 0]
        self.new_vel_idx = 0
        self.num_prev_vels = 3
        self.wheel_radius = 0.06 # 60 mm radius wheel
        self.meters_conversion = 145 * 1.355 / (2 * self.wheel_radius * np.pi) # 120 mm wheel diameter, 145 PPR encoder resolution at gearbox output shaft, 1.355 reduction from belts
        
        # create topics to get feedforward info
        self.left_feedforward_topic = self.create_subscription(
            Float64,
            'left_feedforward',
            self.left_feedforward_callback,
            10)
        
        self.right_feedforward_topic = self.create_subscription(
            Float64,
            'right_feedforward',
            self.right_feedforward_callback,
            10)
        
        self.left_feedforward = 0
        self.right_feedforward = 0
        
        # Create a new service to send data to ros2_control
        self.srv = self.create_service(QuadEncoderData, "encoder_data", self.callback)

    def left_feedforward_callback(self, msg):
        # Convert rad/sec into m/s
        self.left_feedforward = msg.data / self.wheel_radius
        
    def right_feedforward_callback(self, msg):
        # Convert rad/sec into m/s
        self.right_feedforward = msg.data / self.wheel_radius

    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)
        left_position = self.left_enc.read() / self.meters_conversion
        left_velocity = (left_position - self.left_last_position) / 0.01
        right_position = self.right_enc.read() / self.meters_conversion
        right_velocity = (right_position - self.right_last_position) / 0.01 
        
        # Account for different velocities from feedforwards
        left_velocity -= self.left_feedforward
        right_velocity -= self.right_feedforward
        
        # Find the average velocity of the robot
        velocity = np.average([left_velocity, right_velocity])
                
        self.prev_vels[self.new_vel_idx] = velocity
        self.new_vel_idx = (self.new_vel_idx + 1) % self.num_prev_vels
        velocity = np.average(self.prev_vels)
        
        # self.get_logger().info("left: %3.1f right: %3.1f avg: %3.1f" % (left_velocity, right_velocity, velocity))
        
        self.left_last_position = left_position
        self.right_last_position = right_position
        
        response.position = left_position
        response.velocity = velocity

        return response

def main(args=None):
    rclpy.init(args=args)

    node = QuadEncoders()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()