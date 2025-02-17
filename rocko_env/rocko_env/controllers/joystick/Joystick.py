#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist, Vector3 # not sure if needed


class Joystick(Node):

    def __init__(self):
        super().__init__('joystick')

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


        self.server_sock.bind(("0.0.0.0", 1234))
        self.server_sock.listen(1)
        self.get_logger().info("Waiting for joystick connection...")

        self.client_sock, _ = self.server_sock.accept()  # Accept connection
        self.get_logger().info("Joystick connected!")
        self.client_sock.setblocking(False)

        # Set up publisher to command controller
        self.joystick_topic = self.create_publisher(Joy, 'joystick', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.receive_joystick_data)
                
        # # Set up variables to hold data
        # self.left_vel = 0
        # self.right_vel = 0
        # self.desired_robot_body_vector = Twist()



    def receive_joystick_data(self):
        num_axes = 6
        num_buttons = 16
        buffer_size = struct.calcsize(f"{num_axes}f {num_buttons}B")

        try:
            data = self.client_sock.recv(buffer_size)

            # all_data = struct.unpack(f"{num_axes}f {num_buttons}B", data)
            # button_data = struct.unpack_from(f"{num_axes}f {num_buttons}B", data, num_axes)

            unpack = struct.unpack(f"{num_axes}f {num_buttons}B", data)
            msg1 = Joy()
            msg1.axes = list(unpack[:num_axes])
            msg1.buttons = list(unpack[num_axes:])
            # self.get_logger().info(msg1.axes)
            # self.get_logger().info(msg1.buttons)
            # self.get_logger().info(f"\nJoystick Axes: {list(msg1.axes)}")
            # self.get_logger().info(f"\nJoystick Buttons: {list(msg1.buttons)}")

            self.joystick_topic.publish(msg1)
        except BlockingIOError:
            return


    def robot_body_vector_updated(self, msg):
        # Store desired robot body vector
        self.desired_robot_body_vector = msg.twist


def main(args=None):
    rclpy.init(args=args)

    joystick = Joystick()

    rclpy.spin(joystick)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()









