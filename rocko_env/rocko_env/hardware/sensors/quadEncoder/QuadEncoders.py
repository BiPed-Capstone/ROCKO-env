#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
import Encoder

from rocko_interfaces.srv import QuadEncoderData

# Wheel geometry and encoder resolution (both wheels are the same).
WHEEL_RADIUS_M = 0.06  # 60 mm radius wheel (120 mm diameter)
# Encoder counts per metre of wheel travel: 145 counts per revolution of the gearbox output shaft (where the encoder
# sits), times the 1.355 belt reduction between that shaft and the wheel (the original author's figure), divided by
# the wheel's circumference [m]. Which way the 1.355 applies has not been checked against the hardware.
COUNTS_PER_METER = 145 * 1.355 / (2 * WHEEL_RADIUS_M * math.pi)

# Low-pass filter weight of the newest velocity sample: filtered = alpha * new + (1 - alpha) * filtered.
# The filter is applied once per service call, so its time constant is about call_period / alpha:
# ~0.2 s at 100 Hz (balancing) and ~0.4 s at 50 Hz (tank), the update_rate values in bringup/config/
# rocko_controllers.yaml and rocko_tank_mode_controllers.yaml. Changing the update rate changes the lag.
#
# Why the old node read about HALF the true speed (finding U-2): both motor plugins called ONE shared service
# every cycle. The first call saw ~10 ms of motion; the second, a moment later, saw almost none and fed a
# speed of ~0 into the same filter, so the filtered speed settled near half the truth. Now each wheel has its
# own service and its own filter, asked once per cycle.
FILTER_ALPHA = 0.05


class WheelChannel:
    """One wheel's position [m of travel, forward positive] and filtered velocity [m/s].

    Each wheel has its own channel because each wheel is asked once per control cycle by its own
    motor plugin. The velocity is the position change since THIS wheel's previous call, divided by
    the real time between the two calls (not an assumed 0.01 s), so it is right at any update rate
    and when a call is late.
    """

    def __init__(self, filter_alpha=FILTER_ALPHA):
        self.filter_alpha = filter_alpha
        self.last_position_m = None  # None until the first call
        self.last_time_s = None      # time.monotonic() seconds at the previous call
        self.velocity_filtered_m_s = 0.0

    def update(self, position_m, now_s):
        """Return (position [m], filtered velocity [m/s]) for a position read at monotonic time now_s."""
        if self.last_time_s is None:
            # First call: no earlier sample to compare with, so there is no velocity yet. Report 0.0
            # (the robot has not been seen moving), not a division by zero.
            self.last_position_m = position_m
            self.last_time_s = now_s
            return position_m, self.velocity_filtered_m_s

        dt_s = now_s - self.last_time_s
        if dt_s <= 0.0:
            # The clock did not advance: keep the last velocity and keep the OLD reference sample,
            # so the next call measures the motion over the whole elapsed time.
            return position_m, self.velocity_filtered_m_s

        velocity_m_s = (position_m - self.last_position_m) / dt_s
        self.velocity_filtered_m_s = (
            self.filter_alpha * velocity_m_s +
            (1 - self.filter_alpha) * self.velocity_filtered_m_s
        )
        self.last_position_m = position_m
        self.last_time_s = now_s
        return position_m, self.velocity_filtered_m_s


class QuadEncoders(Node):

    def __init__(self):
        super().__init__('node')
        # Handle any hardware initialization here
        self.declare_parameter('left_a_pin', 0)
        left_a_pin: int = self.get_parameter('left_a_pin').get_parameter_value().integer_value
        self.declare_parameter('left_b_pin', 0)
        left_b_pin: int = self.get_parameter('left_b_pin').get_parameter_value().integer_value
        self.declare_parameter('right_a_pin', 0)
        right_a_pin: int = self.get_parameter('right_a_pin').get_parameter_value().integer_value
        self.declare_parameter('right_b_pin', 0)
        right_b_pin: int = self.get_parameter('right_b_pin').get_parameter_value().integer_value

        self.left_enc = Encoder.Encoder(left_a_pin, left_b_pin)
        self.right_enc = Encoder.Encoder(right_b_pin, right_a_pin) # Right encoder is backwards, so we swap pins in code

        self.left = WheelChannel()
        self.right = WheelChannel()

        # One service per wheel. Each motor plugin calls its own (its `encoder_service` hardware
        # parameter in the ros2_control xacro), once per control cycle. A single shared service
        # was called twice per cycle and gave both wheels the left encoder's position (finding U-2).
        self.left_srv = self.create_service(QuadEncoderData, "left_encoder_data", self.left_callback)
        self.right_srv = self.create_service(QuadEncoderData, "right_encoder_data", self.right_callback)

    def left_callback(self, request, response):
        return self.reply(self.left, self.left_enc, response)

    def right_callback(self, request, response):
        return self.reply(self.right, self.right_enc, response)

    def reply(self, channel, encoder, response):
        position_m = encoder.read() / COUNTS_PER_METER
        response.position, response.velocity = channel.update(position_m, time.monotonic())
        return response

def main(args=None):
    rclpy.init(args=args)

    node = QuadEncoders()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
