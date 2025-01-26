#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import Encoder

import numpy as np

from rocko_interfaces.srv import QuadEncoderData

class QuadEncoder(Node):

    def __init__(self):
        super().__init__('node')
        # Handle any hardware initialization here
        self.declare_parameter('service_name', 'quad_encoder')
        service_name: str = self.get_parameter('service_name').get_parameter_value().string_value
        self.declare_parameter('a_pin', '0')
        a_pin: int = self.get_parameter('a_pin').get_parameter_value().integer_value
        self.declare_parameter('b_pin', '0')
        b_pin: int = self.get_parameter('b_pin').get_parameter_value().integer_value
        
        self.enc = Encoder.Encoder(a_pin, b_pin)
        
        self.last_position = 0
        self.meters_conversion = 145 / (0.144 * np.pi) # 144 mm wheel diameter, 145 PPR encoder resolution at gearbox output shaft

        # Create a new service to send data to ros2_control
        self.srv = self.create_service(QuadEncoderData, service_name, self.callback)


    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)
        position = self.enc.read() / self.meters_conversion
        velocity = (position - self.last_position) / 0.01
        response.position = position
        response.velocity = velocity

        self.last_position = position

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