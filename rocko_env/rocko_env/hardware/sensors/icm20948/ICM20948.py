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

        # pitch_offset is in DEGREES and is ADDED to the sensor's pitch in callback(). Its origin
        # and sign convention are not documented in the repo; whoever set 7.6 should record why
        # (for example: the measured angle at which the robot balances).
        self.pitch_offset = 7.6

        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.callback)

    def callback(self, request, response):
        # Finding C-2 (docs/claude-analysis/balance-loop-findings.md): a failed read must never
        # look like "pitch = 0.0" (upright). On failure we answer success=False and NaN angles, so
        # a client that forgets to check `success` still cannot read a believable number.
        # Finding H-3: read self.icm.euler ONCE. Every access is a fresh I2C read, so three
        # accesses gave yaw, roll and pitch from three different instants.
        failure = None
        try:
            euler = self.icm.euler  # (yaw, roll, pitch) in degrees; entries can be None
            if euler is None or any(angle is None for angle in euler):
                failure = "sensor returned no value (euler = " + str(euler) + ")"
            elif euler[0] == euler[1] == euler[2]:
                # Real data has 1/16 deg resolution and three independent angles, so three exactly
                # equal values almost never happen. An all-zero block is what a reset BNO055 or a
                # bus fault can return, and (0, 0, 0) + offset would look like a believable pitch.
                failure = "all three Euler values are identical (euler = " + str(euler) + ")"
            else:
                response.yaw = euler[0]
                response.roll = euler[1]
                response.pitch = euler[2] + self.pitch_offset
                response.success = True
        except Exception as e:
            failure = str(e)

        if failure is not None:
            response.yaw = float('nan')
            response.roll = float('nan')
            response.pitch = float('nan')
            response.success = False
            self.get_logger().warn(
                "Unable to read gyro data this cycle: " + failure, throttle_duration_sec=1.0)

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
