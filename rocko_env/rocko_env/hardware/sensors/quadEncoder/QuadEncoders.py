#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import Encoder

import numpy as np
import math

from rocko_interfaces.srv import QuadEncoderData

class QuadEncoder(Node):

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
        self.right_enc = Encoder.Encoder(right_a_pin, right_b_pin)
        
        self.left_last_position = 0
        self.right_last_position = 0
        self.prev_vels = [0, 0, 0]
        self.new_vel_idx = 0
        self.num_prev_vels = 3
        self.meters_conversion = 145 / (0.12 * np.pi) # 120 mm wheel diameter, 145 PPR encoder resolution at gearbox output shaft

        # Create a new service to send data to ros2_control
        self.srv = self.create_service(QuadEncoderData, "encoder_data", self.callback)


    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)
        left_position = self.left_enc.read() / self.meters_conversion
        left_velocity = (left_position - self.left_last_position) / 0.01
        right_position = self.right_enc.read() / self.meters_conversion
        right_velocity = (right_position - self.right_last_position) / 0.01
        
        velocity = np.average([left_velocity, right_velocity])
        
        self.prev_vels[self.new_vel_idx] = velocity
        self.new_vel_idx = (self.new_vel_idx + 1) % self.num_prev_vels
        
        self.last_left_position = left_position
        self.last_right_position = right_position
        
        response.position = left_position
        response.velocity = np.average(self.prev_vels)

        return response

def main(args=None):
    rclpy.init(args=args)

    node = QuadEncoder()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()