#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
import adafruit_icm20x

import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q
from ahrs import Quaternion


from rocko_interfaces.srv import Icm20948Data

# Documentation for gyro lib: https://docs.circuitpython.org/projects/icm20x/en/latest/
class ICM20948(Node):

    def __init__(self):
        # Create a new service called /icm20948 for posting IMU info
        super().__init__('icm20948_node')
        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.imu_callback)

        # Initialize the gyro board
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.icm = adafruit_icm20x.ICM20948(i2c)

        # Set up AHRS equation
        self.ekf = EKF()
        self.num_samples = 1000              # Assuming sensors have 1000 samples each
        self.current_sample_idx = 1
        self.Q = np.zeros((self.num_samples, 4))  # Allocate array for quaternions
        self.Q[0] = acc2q(self.icm.acceleration)

    def imu_callback(self, request, response):
        # Grab IMU data and send it to the topic
        self.Q[self.current_sample_idx] = self.ekf.update(self.Q[self.current_sample_idx - 1], self.icm.gyro, self.icm.acceleration, self.icm.magnetic)

        euler_angles = np.degrees(Quaternion(self.Q[self.current_sample_idx]).to_angles())
        response.x = euler_angles[0]
        response.y = euler_angles[1]
        response.z = euler_angles[2]

        self.current_sample_idx += 1

        if (self.current_sample_idx == self.num_samples):
            self.current_sample_idx = 1

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