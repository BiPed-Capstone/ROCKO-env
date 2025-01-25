#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
from adafruit_seesaw import seesaw, rotaryio

import numpy as np

from rocko_interfaces.srv import Pid4991Data

class PID4991(Node):

    def __init__(self, service_name, addr:int):
        # Handle any hardware initialization here
        i2c = board.I2C()  # uses board.SCL and board.SDA
        self.s = seesaw.Seesaw(i2c, addr)

        seesaw_product = (self.s.get_version() >> 16) & 0xFFFF
        print("Found product {}".format(seesaw_product))
        if seesaw_product != 4991:
            print("Wrong firmware loaded?  Expected 4991")
            
        self.s.enable_encoder_interrupt()

        # self.encoder = rotaryio.IncrementalEncoder(s)
        # self.last_position = 0
        self.meters_conversion = 53 / (0.144 * np.pi) # 144 mm wheel diameter, 145.1 PPR encoder resolution at gearbox output shaft

        # Create a new service to send data to ros2_control
        super().__init__('node')
        self.srv = self.create_service(Pid4991Data, service_name, self.callback)


    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)
        position = self.s.encoder_position() / self.meters_conversion
        velocity = self.s.encoder_delta() / self.meters_conversion / 0.01
        response.position = position
        response.velocity = velocity
        
        # position = self.encoder.position / self.meters_conversion
        # response.position = position
        # response.velocity = (position - self.last_position) / 0.01
        # self.last_position = position

        self.get_logger().info("raw pos: " + str(self.s.encoder_position()))

        return response

def main(args=None):
    rclpy.init(args=args)

    node = PID4991(service_name="left_pid4991_data", addr=0x36)

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()