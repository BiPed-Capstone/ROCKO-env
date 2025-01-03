#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import board
# import adafruit_icm20x

from rocko_interfaces.srv import Icm20948Data


class MinimalPublisher(Node):

    def __init__(self):
        # Create a new service called /icm20948 for posting IMU info
        super().__init__('icm20948_node')
        self.srv = self.create_service(Icm20948Data, 'icm20948', self.imu_callback)

        # Initialize the gyro board
        # TODO: Get lib to work and init board here
        # i2c = board.I2C()   # uses board.SCL and board.SDA
        # self.icm = adafruit_icm20x.ICM20649(i2c)

    def imu_callback(self, request, response):
        # Grab IMU data and send it to the topic
        # TODO: Call gyro object to get data for message
        response.x = 0
        response.y = 0
        response.z = 0

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()