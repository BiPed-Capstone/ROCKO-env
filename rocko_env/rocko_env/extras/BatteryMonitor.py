#!/usr/bin/env python3
"""Battery monitor: reads the battery voltage from a TI INA226 50 times a second and publishes it.

Optional (docs/architecture/robot-extras.md): with no sensor fitted, or a broken one, it logs the error, reports
it on /diagnostics (the dashboard and the screen show it), tries again every health.RETRY_PERIOD_S, and changes
nothing else. Wiring and Pi setup: docs/guides/fit-the-extras.md.

Publishes:
  /rocko/battery_raw  sensor_msgs/BatteryState, EVERY reading (sample_rate_hz, default 50 Hz): voltage [V, not
                      smoothed], current [A, NaN unless shunt_ohms is set], stamped when it was read. For recording
                      and plotting brown-outs (short sags while the motors pull hard) against the control effort.
  /rocko/battery      sensor_msgs/BatteryState, once a second: voltage [V, smoothed], percentage [0..1, rough],
                      current. The supervisor reads the voltage to refuse an enable at battery.CRITICAL_V.
  /diagnostics        "rocko/battery", once a second: OK, WARN (low) or ERROR (no reading), with voltage, percent,
                      level, the measured sample rate and the count of skipped readings as values.

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
        rate_hz = float(self.declare_parameter("sample_rate_hz", battery.DEFAULT_SAMPLE_RATE_HZ).value)
        if not 0.0 < rate_hz <= battery.MAX_SAMPLE_RATE_HZ:
            self.get_logger().warn(f"battery monitor: sample_rate_hz {rate_hz} is outside 0 to "
                                   f"{battery.MAX_SAMPLE_RATE_HZ:.0f}; using {battery.DEFAULT_SAMPLE_RATE_HZ:.0f}")
            rate_hz = battery.DEFAULT_SAMPLE_RATE_HZ

        self.sensor = None              # an open Ina226, or None while there is no working sensor
        self.next_try_s = 0.0           # [monotonic s] when to try opening it again
        self.smoothed_v = None
        self.latest_v = None            # the newest reading [V, not smoothed], or None
        self.latest_a = math.nan        # the newest current [A], NaN unless shunt_ohms is set
        self.level = battery.LEVEL_OK
        self.last_read_s = None
        self.bad_readings = battery.BadReadings()
        self.good_since_report = 0      # readings published since the last report: the measured sample rate
        self.status = health.report("battery", health.ERROR, "starting")
        self.logged = None              # the last error logged, so a lasting fault is logged once, not every second

        self.raw_pub = self.create_publisher(BatteryState, "/rocko/battery_raw", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/rocko/battery", 5)
        self.health_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        # Two timers: one reads the chip at the sample rate, the other summarises once a second for the supervisor,
        # the dashboard and the screen, which do not need 50 messages a second.
        self.create_timer(1.0 / rate_hz, self.sample)
        self.create_timer(health.REPORT_PERIOD_S, self.report)

    def fail(self, message):
        """Report and log an error, keep running, and try the sensor again later."""
        if self.sensor is not None:
            self.sensor.close()
            self.sensor = None
        self.smoothed_v = None          # a reading from before the fault is not carried over
        self.latest_v = None
        self.bad_readings.good()        # the next fault gets its own GIVE_UP_AFTER_S
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
            # Make sure it is an INA226: the arithmetic in battery.py is that chip's (an INA219, for one, counts
            # 4 mV per step, not 1.25 mV, and would read wrong voltages without any error).
            maker = sensor.read_u16(battery.REG_MANUFACTURER_ID)
            die = sensor.read_u16(battery.REG_DIE_ID)
            if maker != battery.MANUFACTURER_TI or die != battery.DIE_ID_INA226:
                sensor.close()
                self.fail(f"the chip at 0x{self.address:02x} on {self.device} is not an INA226 "
                          f"(manufacturer id 0x{maker:04x}, die id 0x{die:04x}; an INA226 says 0x5449, 0x2260)")
                return
            sensor.write_u16(battery.REG_CONFIG, battery.CONFIG_AVG4_CONTINUOUS)
        except OSError as error:
            sensor.close()
            self.fail(f"no answer from 0x{self.address:02x} on {self.device}: {error.strerror or error}")
            return
        self.sensor = sensor
        self.get_logger().info(f"battery monitor: INA226 found on {self.device} at 0x{self.address:02x}")

    def sample(self):
        """Every 1 / sample_rate_hz: read the chip once, if there is one, and publish the reading."""
        if self.sensor is None and time.monotonic() >= self.next_try_s:
            self.open_sensor()
        if self.sensor is not None:
            self.read_once()

    def bad_reading(self, message):
        """Skip this reading. Only when bad readings have gone on for battery.GIVE_UP_AFTER_S is it a fault."""
        if self.bad_readings.bad(time.monotonic()):
            self.fail(message)

    def read_once(self):
        try:
            volts = battery.bus_voltage_v(self.sensor.read_u16(battery.REG_BUS_VOLTAGE))
            amps = (battery.shunt_current_a(self.sensor.read_u16(battery.REG_SHUNT_VOLTAGE), self.shunt_ohms)
                    if self.shunt_ohms > 0 else math.nan)
        except OSError as error:
            self.bad_reading(f"read failed: {error.strerror or error}")
            return
        # Stamped right after the read, not when a subscriber gets it: Foxglove's battery layout plots it at this
        # time (headerStamp), so a late delivery does not shift the sag.
        stamp = self.get_clock().now().to_msg()
        if not battery.PLAUSIBLE_MIN_V <= volts <= battery.PLAUSIBLE_MAX_V:
            # Not published: a 0 V "battery" would look like a dead battery and block enabling for a wiring fault.
            self.bad_reading(f"implausible reading {volts:.2f} V: check that VBUS (or IN+ and IN-) reaches the "
                             "battery +")
            return
        self.bad_readings.good()

        msg = BatteryState()
        msg.header.stamp = stamp
        msg.voltage = volts
        msg.current = amps
        msg.percentage = math.nan       # not estimated from one unsmoothed reading; see /rocko/battery
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        self.raw_pub.publish(msg)
        self.good_since_report += 1

        now = time.monotonic()
        dt = 0.0 if self.last_read_s is None else now - self.last_read_s
        self.last_read_s = now
        # The filter runs on every reading; the level, the percentage and the reports only once a second, in
        # report(): nobody needs them faster, and the buzzer and the screen act on the level.
        self.smoothed_v = battery.smooth(self.smoothed_v, volts, dt)
        self.latest_v, self.latest_a = volts, amps

    def report(self):
        """Once a second: the smoothed voltage on /rocko/battery and this extra's health on /diagnostics."""
        good = self.good_since_report
        self.good_since_report = 0
        if self.sensor is not None and good == 0:
            # Open, but every read this second failed (fewer than GIVE_UP_AFTER_S of them, or fail() would have run).
            # /rocko/battery stays quiet, so the supervisor's age check soon treats the voltage as unknown, and the
            # report says so: an old "OK" with a fresh stamp would be a safe-looking value.
            self.status = health.report("battery", health.WARN, "no good reading in the last second",
                                        {"skipped_readings": self.bad_readings.count})
        elif self.sensor is not None and self.smoothed_v is not None:
            self.publish_smoothed(good / health.REPORT_PERIOD_S)
        self.health_pub.publish(health.to_message(self.status))

    def publish_smoothed(self, rate_hz):
        self.level = battery.level(self.smoothed_v, self.level)
        percent = battery.percent_from_voltage(self.smoothed_v)

        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self.smoothed_v
        msg.current = self.latest_a
        msg.percentage = percent / 100.0
        msg.design_capacity = CAPACITY_AH
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        msg.present = True
        self.battery_pub.publish(msg)

        level_number = {battery.LEVEL_OK: health.OK}.get(self.level, health.WARN)
        text = {battery.LEVEL_OK: "ok", battery.LEVEL_WARN: "low", battery.LEVEL_CRITICAL: "critical: charge it"}
        self.status = health.report("battery", level_number, f"{self.smoothed_v:.2f} V, {text[self.level]}",
                                    {"voltage": f"{self.smoothed_v:.3f}", "percent": f"{percent:.0f}",
                                     "level": self.level,
                                     "raw_voltage": f"{self.latest_v:.3f}",   # the newest reading, unsmoothed
                                     "sample_rate_hz": f"{rate_hz:.0f}",
                                     "skipped_readings": self.bad_readings.count})
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
