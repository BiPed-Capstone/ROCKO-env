#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
import adafruit_icm20x

import numpy as np
from time import sleep
from ahrs.filters import Complementary
from ahrs.common.orientation import acc2q
from ahrs import Quaternion


from rocko_interfaces.srv import Pid4991Data

class PID4991(Node):

    def __init__(self):
        # Handle any hardware initialization here

        # Create a new service to send data to ros2_control
        super().__init__('node')
        self.srv = self.create_service(Pid4991Data, 'pid4991_data', self.callback)


    def callback(self, request, response):
        # Interact with hardware here based on info in request (if there is any)

        # Put data into response according to the service type declared in init

        return response

def main(args=None):
    rclpy.init(args=args)

    node = PID4991()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()