#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
import board
import adafruit_icm20x
from adafruit_icm20x import AccelRange, GyroRange

import numpy as np
from time import sleep
from ahrs.filters import Madgwick
from ahrs.common.orientation import acc2q
from ahrs import Quaternion

import board
import adafruit_bno055


from rocko_interfaces.srv import Icm20948Data

# Documentation for gyro lib: https://docs.circuitpython.org/projects/icm20x/en/latest/
class ICM20948(Node):

    def __init__(self):
        super().__init__('icm20948_node')
        # Initialize the gyro board
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.icm = adafruit_bno055.BNO055_I2C(i2c)

        self.pitch_offset = 7.6

        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.callback)

    def callback(self, request, response):
        try:
            response.yaw = self.icm.euler[0]
            response.roll = self.icm.euler[1]
            response.pitch = self.icm.euler[2] + self.pitch_offset
        except Exception as e:
            self.get_logger().warn("Unable to read gyro data this cycle: " + str(e))
                
        return response

def main(args=None):
    rclpy.init(args=args)

    imu = ICM20948()

    rclpy.spin(imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
