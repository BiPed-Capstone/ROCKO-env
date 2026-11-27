#!/usr/bin/env python3
"""Battery monitor: reads the battery voltage from a TI INA226 once a second and publishes it.

Optional (docs/architecture/robot-extras.md): with no sensor fitted, or a broken one, it logs the error, reports
it on /diagnostics (the dashboard and the screen show it), tries again every health.RETRY_PERIOD_S, and changes
nothing else. Wiring and Pi setup: docs/guides/fit-the-extras.md.

Publishes:
  /rocko/battery  sensor_msgs/BatteryState: voltage [V, smoothed], percentage [0..1, rough], current [A, NaN unless
                  shunt_ohms is set]. The supervisor reads the voltage to refuse an enable at battery.CRITICAL_V.
  /diagnostics    "rocko/battery": OK, WARN (low) or ERROR (no reading), with voltage, percent and level as values.

Why its own I2C bus: the IMU is read on bus 1 a hundred times a second inside the control loop, and a second device
on that bus can make the IMU read wait for it. The INA226 sits on a separate bus (default /dev/i2c-3), so it cannot.
Not on the 100 Hz path: a separate process.
"""
import fcntl
import math
import os
import time

import rclpy
import rclpy.executors
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from rocko_env.extras import battery, health

I2C_SLAVE = 0x0703          # Linux ioctl: "the next reads and writes on this file go to this chip address"
CAPACITY_AH = 10.0          # [Ah] the battery's rating, for BatteryState.design_capacity


class Ina226:
    """The INA226 over Linux's /dev/i2c-N, with the standard library only. Every call may raise OSError."""

    def __init__(self, device, address):
        self.fd = os.open(device, os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
        except OSError:
            os.close(self.fd)
            raise

    def read_u16(self, register):
        os.write(self.fd, bytes([register]))       # point the chip at the register
        data = os.read(self.fd, 2)
        if len(data) != 2:
            raise OSError(f"short read from register 0x{register:02x}")
        return (data[0] << 8) | data[1]            # the INA226 sends the high byte first

    def write_u16(self, register, value):
        os.write(self.fd, bytes([register, value >> 8, value & 0xFF]))

    def close(self):
        os.close(self.fd)


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__("battery_monitor")
        self.device = self.declare_parameter("i2c_device", "/dev/i2c-3").value
        self.address = int(self.declare_parameter("i2c_address", battery.INA226_ADDRESS).value)
        # [ohm] 0 (the default) = voltage only. Set it only if the motor current really flows through the shunt;
        # the breakout board's own 0.1 ohm shunt saturates at 0.8 A, far below what the motors draw.
        self.shunt_ohms = float(self.declare_parameter("shunt_ohms", 0.0).value)

        self.sensor = None              # an open Ina226, or None while there is no working sensor
        self.next_try_s = 0.0           # [monotonic s] when to try opening it again
        self.smoothed_v = None
        self.level = battery.LEVEL_OK
        self.last_read_s = None
        self.status = health.report("battery", health.ERROR, "starting")
        self.logged = None              # the last error logged, so a lasting fault is logged once, not every second

        self.battery_pub = self.create_publisher(BatteryState, "/rocko/battery", 5)
        self.health_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_timer(health.REPORT_PERIOD_S, self.tick)

    def fail(self, message):
        """Report and log an error, keep running, and try the sensor again later."""
        if self.sensor is not None:
            self.sensor.close()
            self.sensor = None
        self.smoothed_v = None          # a reading from before the fault is not carried over
        self.next_try_s = time.monotonic() + health.RETRY_PERIOD_S
        self.status = health.report("battery", health.ERROR, message)
        if message != self.logged:
            self.logged = message
            self.get_logger().error(f"battery monitor: {message} (the robot runs without it; retrying every "
                                    f"{health.RETRY_PERIOD_S:.0f} s)")

    def open_sensor(self):
        try:
            sensor = Ina226(self.device, self.address)
        except OSError as error:
            self.fail(f"cannot open {self.device} address 0x{self.address:02x}: {error.strerror or error}")
            return
        try:
            maker = sensor.read_u16(battery.REG_MANUFACTURER_ID)
            if maker != battery.MANUFACTURER_TI:
                sensor.close()
                self.fail(f"the chip at 0x{self.address:02x} on {self.device} is not an INA226 "
                          f"(manufacturer id 0x{maker:04x})")
                return
            sensor.write_u16(battery.REG_CONFIG, battery.CONFIG_AVG16_CONTINUOUS)
        except OSError as error:
            sensor.close()
            self.fail(f"no answer from 0x{self.address:02x} on {self.device}: {error.strerror or error}")
            return
        self.sensor = sensor
        self.get_logger().info(f"battery monitor: INA226 found on {self.device} at 0x{self.address:02x}")

    def tick(self):
        if self.sensor is None and time.monotonic() >= self.next_try_s:
            self.open_sensor()
        if self.sensor is not None:
            self.read_once()
        self.health_pub.publish(health.to_message(self.status))

    def read_once(self):
        try:
            volts = battery.bus_voltage_v(self.sensor.read_u16(battery.REG_BUS_VOLTAGE))
            amps = (battery.shunt_current_a(self.sensor.read_u16(battery.REG_SHUNT_VOLTAGE), self.shunt_ohms)
                    if self.shunt_ohms > 0 else math.nan)
        except OSError as error:
            self.fail(f"read failed: {error.strerror or error}")
            return
        if not battery.PLAUSIBLE_MIN_V <= volts <= battery.PLAUSIBLE_MAX_V:
            # Not published: a 0 V "battery" would look like a dead battery and block enabling for a wiring fault.
            self.fail(f"implausible reading {volts:.2f} V: check that VBUS (or IN+ and IN-) reaches the battery +")
            return

        now = time.monotonic()
        dt = 0.0 if self.last_read_s is None else now - self.last_read_s
        self.last_read_s = now
        self.smoothed_v = battery.smooth(self.smoothed_v, volts, dt)
        self.level = battery.level(self.smoothed_v, self.level)
        percent = battery.percent_from_voltage(self.smoothed_v)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.smoothed_v
        msg.current = amps
        msg.percentage = percent / 100.0
        msg.design_capacity = CAPACITY_AH
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        self.battery_pub.publish(msg)

        level_number = {battery.LEVEL_OK: health.OK}.get(self.level, health.WARN)
        text = {battery.LEVEL_OK: "ok", battery.LEVEL_WARN: "low", battery.LEVEL_CRITICAL: "critical: charge it"}
        self.status = health.report("battery", level_number, f"{self.smoothed_v:.2f} V, {text[self.level]}",
                                    {"voltage": f"{self.smoothed_v:.3f}", "percent": f"{percent:.0f}",
                                     "level": self.level, "raw_voltage": f"{volts:.3f}"})
        self.logged = None   # a later fault is logged again


def main():
    rclpy.init()
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.sensor is not None:
            node.sensor.close()


if __name__ == "__main__":
    main()
