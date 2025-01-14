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


from rocko_interfaces.srv import Icm20948Data

# Documentation for gyro lib: https://docs.circuitpython.org/projects/icm20x/en/latest/
class ICM20948(Node):

    def __init__(self):
        # Initialize the gyro board
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.icm = adafruit_icm20x.ICM20948(i2c)

        # Calibrate gyro before opening service
        # Take 100 samples and feed them to ekf algorithm to give us the initial position
        num_calib_samples = 100
        gyr_arr = np.zeros((num_calib_samples, 3))
        acc_arr = np.zeros((num_calib_samples, 3))
        mag_arr = np.zeros((num_calib_samples, 3))
        for i in range(0, num_calib_samples):
            # Add data to sample arrays
            gyr_arr[i] = self.icm.gyro
            acc_arr[i] = self.icm.acceleration
            mag_arr[i] = self.icm.magnetic
            # Wait for 10ms for new data to be gathered
            sleep(0.01)

        # Calculate median
        self.initial_q = np.median(Complementary(gyr_arr, acc_arr, mag_arr).Q, axis=0)

        # Create a new service called /icm20948_data for posting IMU positional data
        super().__init__('icm20948_node')
        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.imu_callback)


    def imu_callback(self, request, response):
        g = np.array([self.icm.gyro])
        a = np.array([self.icm.acceleration])
        m = np.array([self.icm.magnetic])
        current_q = Complementary(g, a, m).Q[0]
        diff = np.subtract(current_q, self.initial_q)
        angles = Quaternion(diff).to_angles()

        # Prepare data for sending
        response.yaw = angles[0]
        response.roll = angles[1]
        response.pitch = angles[2]

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