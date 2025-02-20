#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
import board
import adafruit_icm20x

import numpy as np
from time import sleep
from ahrs.filters import Madgwick
from ahrs.common.orientation import acc2q
from ahrs import Quaternion


from rocko_interfaces.srv import Icm20948Data

# Documentation for gyro lib: https://docs.circuitpython.org/projects/icm20x/en/latest/
class ICM20948(Node):

    def __init__(self):
        super().__init__('icm20948_node')
        # Initialize the gyro board
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.icm = adafruit_icm20x.ICM20948(i2c)

        # Calibrate sens before opening service
        self.use_hard_offsets = False # set to True if you want to loop in external calibrations

        # Take 100 samples and feed them to ekf algorithm to give us the initial position
        num_calib_samples = 100
        gyr_arr = np.zeros((num_calib_samples, 3))
        acc_arr = np.zeros((num_calib_samples, 3))
        for i in range(0, num_calib_samples):
            # Add data to sample arrays
            gyr_arr[i] = self.icm.gyro
            acc_arr[i] = self.icm.acceleration
            # Wait for 10ms for new data to be gathered
            sleep(0.01)

        # Calculate median
        self.madgwick = Madgwick(gyr_arr, acc_arr)
        self.prev_q = np.median(self.madgwick.Q, axis=0)
        self.zero_q = self.prev_q

        # Check filesystem for accelerometer calibration data
        path = os.path.join('calibration', 'hard_offset')
        self.calibration_results = []
        try:
            # If cal data is there, we will interpolate it for offset
            # Extract calibration values from cal file (newline deliniated)
            with open(path) as file:
                for i in file:
                    self.calibration_results.append(i)
        except:
            # self.get_logger().warn('No accelerometer calibration data found, proceeding anyway.')
            for i in range(3):
                self.calibration_results.append(0)

        # Create a new service called /icm20948_data for posting IMU positional data
        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.imu_callback)


    def imu_callback(self, request, response):
        g = np.array(self.icm.gyro)
        a = np.array(self.icm.acceleration)

        if self.use_hard_offsets:
            for i in range(3):
                a[i] = a[i] + self.calibration_results[i]

        current_q = self.madgwick.updateIMU(q=self.prev_q, gyr=g, acc=a)
        # current_q = np.subtract(current_q, self.zero_q)
        # self.get_logger().info("cur: " + str(current_q) + " zero: " + str(self.zero_q))
        self.prev_q = current_q
        angles = np.degrees(Quaternion(current_q).to_angles())
        
        if (angles[0] > 0): 
            angles[0] -= 180
        else: 
            angles[0] += 180

        # Prepare data for sending
        response.yaw = angles[2]
        response.roll = angles[1]
        response.pitch = angles[0] - 1.0
                
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