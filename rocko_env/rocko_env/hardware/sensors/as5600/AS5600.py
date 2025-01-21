#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import board
from adafruit_bus_device.i2c_device import I2CDevice

import numpy as np
from rocko_interfaces.srv import As5600Data

DEVICE_AS5600 = 0x36 # Device I2C address

class AS5600(Node):
    # See pg 18 for reg map (https://www.mouser.com/datasheet/2/588/AS5600_DS000365_5-00-1877365.pdf)
    def __init__(self, serviceName:str):
        i2c = board.I2C()   # uses board.SCL and board.SDA
        self.device = I2CDevice(i2c, DEVICE_AS5600)

        # Create a new service called /icm20948_data for posting IMU positional data
        super().__init__(serviceName + "_node")
        self.srv = self.create_service(As5600Data, serviceName, self.service_callback)

    def read_angle(self): # Read angle (0-360)
        read_bytes = bytearray(2)
        self.device.write_then_readinto(bytes([0x0E]), read_bytes)
        return (read_bytes[0]<<8) | read_bytes[1]

    def read_raw_angle(self): # Raw angle (0-360 represented as 0-4096)
        read_bytes = bytearray(2)
        self.device.write_then_readinto(bytes([0x0C]), read_bytes)
        return (read_bytes[0]<<8) | read_bytes[1]

    def service_callback(self, request, response):
        # Read data from encoder and package it in response
        response.angle = self.read_angle()

        return response

def main(args=None):
    rclpy.init(args=args)

    node = AS5600(serviceName=args[0])

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()