#!/usr/bin/env python3
import math
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

# [deg/s] The BNO055 gyro's full scale in its fusion modes is +/-2000 deg/s, so a larger pitch rate
# cannot be a real measurement and is sent as NaN. MUST equal kMaxAbsPitchRateDegS in
# include/rocko_env/ICM20948.hpp, which applies the same limit in the C++ plugin. The node checks it
# too so that a rate that is out of range from the very first read is logged here: the plugin only
# warns about bad rates after it has seen a good one (finding M-7).
MAX_ABS_PITCH_RATE_DEG_S = 2000.0

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

        # Finding H-6: the pitch controller's D term should use the gyro's measured rate, not a
        # derivative of the (delayed) fused angle. self.icm.gyro is (x, y, z) in RADIANS PER SECOND
        # in the SENSOR's frame (Adafruit driver docstring). The BNO055 itself counts 1 LSB = 1/16
        # deg/s; the driver converts. UNITS CAVEAT: Adafruit driver versions before 5.0 returned
        # deg/s, and with one of those degrees() below makes the rate 57x too large. Check the
        # installed version, and the robot procedure must check the magnitude (tilt the robot about
        # 90 deg in about 1 s: expect tens of deg/s, not thousands).
        # We export one axis of it as pitch_rate_deg_s:
        #     pitch_rate_deg_s = pitch_rate_sign * degrees(gyro[pitch_rate_axis])
        # UNVERIFIED: establish both on the robot by tilting it; an inverted D term is worse than
        # none. The defaults are a GUESS: axis 0 (gyro X) because the BNO055 datasheet describes its
        # Euler pitch as the rotation about the sensor's X axis, and sign +1 because nothing here says
        # otherwise. Procedure: docs/claude-analysis/bno055-improvements.md, section 1.
        #
        # read_pitch_rate (default False, finding M-7): whether to read the gyro at all.
        # rocko_controllers.yaml does not use pitch_rate today, and the gyro read is a second I2C
        # transaction inside the blocking service call of every 10 ms cycle (finding C-1), so it
        # costs loop time for nothing. While False the gyro is never read and pitch_rate_deg_s is
        # NaN ("no rate"), which the C++ plugin holds as NaN. Turn it on together with the gated
        # YAML change that makes pitch_pid_controller use pitch_rate (finding H-6).
        self.read_pitch_rate = self.declare_parameter('read_pitch_rate', False).value
        self.pitch_rate_axis = self.declare_parameter('pitch_rate_axis', 0).value  # 0, 1 or 2
        self.pitch_rate_sign = self.declare_parameter('pitch_rate_sign', 1).value  # +1 or -1
        if self.pitch_rate_axis not in (0, 1, 2):
            raise ValueError("pitch_rate_axis must be 0, 1 or 2, got " + str(self.pitch_rate_axis))
        if self.pitch_rate_sign not in (1, -1):
            raise ValueError("pitch_rate_sign must be 1 or -1, got " + str(self.pitch_rate_sign))
        if self.read_pitch_rate:
            self.get_logger().warn(
                "pitch_rate = %+d * gyro[%d]: axis and sign are UNVERIFIED on the robot (finding H-6)"
                % (self.pitch_rate_sign, self.pitch_rate_axis))
        else:
            self.get_logger().info(
                "read_pitch_rate is false: the gyro is not read and pitch_rate_deg_s is sent as NaN")

        self.srv = self.create_service(Icm20948Data, 'icm20948_data', self.callback)

    def callback(self, request, response):
        # Finding C-2 (docs/claude-analysis/balance-loop-findings.md): a failed read must never
        # look like "pitch = 0.0" (upright). On failure we answer success=False and NaN angles, so
        # a client that forgets to check `success` still cannot read a believable number.
        # Finding H-3: read self.icm.euler ONCE. Every access is a fresh I2C read, so three
        # accesses gave yaw, roll and pitch from three different instants.
        # `success` is about the ANGLES only. The pitch rate is judged on its own (see
        # read_pitch_rate_deg_s): a bad gyro value makes the rate NaN, but never throws away a good
        # pitch, which the balance loop needs far more than the D term's rate.
        # ORDER MATTERS (section 6: every extra I2C transaction ages the pitch). The gyro read costs
        # one more transaction plus Python overhead, and it happens before the reply goes out. Read
        # it FIRST so the euler read, whose pitch is the signal the balance loop needs most, is the
        # freshest sample in the reply. The real fix is one 12-byte burst read of registers
        # 0x14-0x1F (gyro and Euler together), which belongs to the C-1 migration.
        # With read_pitch_rate False (the default) there is no gyro read at all, only the euler one.
        if self.read_pitch_rate:
            pitch_rate_deg_s = self.read_pitch_rate_deg_s()
        else:
            pitch_rate_deg_s = float('nan')

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

        response.pitch_rate_deg_s = pitch_rate_deg_s
        return response

    def read_pitch_rate_deg_s(self):
        """The pitch rate [deg/s] from the gyro, or NaN if this cycle's gyro read failed (None, an
        exception, a non-finite value, or a rate beyond +/-MAX_ABS_PITCH_RATE_DEG_S). A gyro fault
        therefore shows up as NaN in pitch_rate_deg_s only; the angles and `success` are not touched.

        There is deliberately NO "all three values identical" rule here, unlike the angles: a gyro
        that is truly still reads (0, 0, 0), and that is a real measurement.

        This is a second sensor read, a few milliseconds before the angles (callback() reads it
        first, so the pitch is the fresher of the two), so the rate and the angles are not one
        snapshot. Reading Euler angles and gyro in one 12-byte I2C burst would
        fix that, but needs the hardware to test; it belongs to the C-1 IMU migration (stage 6).
        """
        failure = None
        try:
            gyro = self.icm.gyro  # (x, y, z) in rad/s, sensor frame; entries can be None
            rate_rad_s = None if gyro is None else gyro[self.pitch_rate_axis]
            if rate_rad_s is None or not math.isfinite(rate_rad_s):
                failure = "gyro returned no usable value (gyro = " + str(gyro) + ")"
        except Exception as e:
            failure = str(e)
        if failure is None:
            rate_deg_s = self.pitch_rate_sign * math.degrees(rate_rad_s)
            if abs(rate_deg_s) > MAX_ABS_PITCH_RATE_DEG_S:
                failure = ("rate %.0f deg/s is beyond the gyro's +/-%.0f deg/s range"
                           % (rate_deg_s, MAX_ABS_PITCH_RATE_DEG_S))
        if failure is not None:
            self.get_logger().warn(
                "Pitch rate unavailable this cycle (sent as NaN; the angles are unaffected): "
                + failure, throttle_duration_sec=1.0)
            return float('nan')
        return rate_deg_s

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
