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
        # self.icm = adafruit_icm20x.ICM20948(i2c)
        # self.icm.gyro_data_rate_divisor = 0
        # self.icm.accelerometer_data_rate_divisor = 0
        # self.icm.accelerometer_range = AccelRange.RANGE_8G
        # self.icm.gyro_range = GyroRange.RANGE_1000_DPS
        
        # TODO: Replace this 0 with the one printed out by the calibration routine
        # self.zero_q = [-7.59995762e-04, 2.90257081e-04, 9.99942906e-01, -1.06547126e-02]
        
        # # Load in calibration data
        # gyr_arr = np.zeros((1000, 4))
        # acc_arr = np.zeros((1000, 4))
        # path = "rocko_env/rocko_env/hardware/sensors/icm20948/calibration/gyro_acc_data.txt"
        # with open(path) as file:
        #     # Each line has one sample, gyro/accel data delineated by ,
        #     idx = 0
        #     for line in file:
        #         data = line.split(",")
        #         gyro_data = data[0]
        #         acc_data = data[1]
        #         gyr_arr[idx] = np.fromstring(gyro_data[1:len(gyro_data) - 1], sep=",")
        #         acc_arr[idx] = np.fromstring(acc_data[1:len(acc_data) - 1], sep=",")
        #         idx += 1

        # self.madgwick = Madgwick(gyr_arr, acc_arr, gain=0.04)
        # self.prev_q = self.zero_q
        # self.zero_q = np.degrees(Quaternion(self.zero_q).to_angles())
        
        # if (self.zero_q[0] > 0): 
        #     self.zero_q[0] -= 180
        # else: 
        #     self.zero_q[0] += 180
        while not self.icm.calibrated:
            pass

        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.imu_callback)

    def imu_callback(self, request, response):
        try:
            # g = np.array(self.icm.gyro)
            # a = np.array(self.icm.acceleration)
            
            # # convert gyro from degrees/sec to rad/sec

            # current_q = self.madgwick.updateIMU(q=self.prev_q, gyr=g, acc=a)        
                
            # self.prev_q = current_q
            # angles = np.degrees(Quaternion(current_q).to_angles())
            
            # if (angles[0] > 0): 
            #     angles[0] -= 180
            # else: 
            #     angles[0] += 180

            # # Invert angle
            # angles[0] *= -1
            # # Prepare data for sending
            # response.yaw = angles[2] - self.zero_q[2]
            # response.roll = angles[1] - self.zero_q[1]
            # response.pitch = angles[0] - self.zero_q[0] + 10.0
            response.pitch = self.icm.euler[2]
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
