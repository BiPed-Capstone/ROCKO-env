#!/usr/bin/env python3
"""Host test for the IMU node's service reply (ICM20948.py; findings C-2 and H-6). No ROS, no
sensor, no Raspberry Pi needed.

Run from the repo root:  python3 -I tools/imu_node_test.py

It loads ICM20948.py with small stand-ins for rclpy, the Adafruit/ahrs libraries and the service
type, gives it a fake BNO055, and checks:
  * by default (read_pitch_rate false, finding M-7) the gyro is NEVER read: one euler read per
    reply, pitch_rate_deg_s is NaN, the angles and success are unaffected, nothing is warned;
With read_pitch_rate true:
  * good angles + good gyro: success, pitch includes pitch_offset, rate = sign * degrees(gyro[axis]);
  * a bad gyro (None, an exception, NaN) makes ONLY pitch_rate_deg_s NaN: the good angles and
    success=True are kept;
  * so does a rate beyond +/-2000 deg/s (the gyro's range), and it is logged by the node;
  * the gyro is read BEFORE the Euler angles, so the pitch is the freshest sample (once each);
  * a still gyro reading (0, 0, 0) is a real measurement (rate 0.0), not a failure;
  * bad angles (None, all three identical, an exception) give success=False and NaN angles,
    whatever the gyro says.
What it cannot show: the real driver's units or timing, the I2C bus, or the robot. Tier 0 only.
"""
import importlib.util
import math
import pathlib
import sys
import types
from types import SimpleNamespace

REPO = pathlib.Path(__file__).resolve().parent.parent
NODE = REPO / "rocko_env" / "rocko_env" / "hardware" / "sensors" / "icm20948" / "ICM20948.py"


class FakeLogger:
    def __init__(self):
        self.lines = []

    def warn(self, text, **kwargs):
        self.lines.append(("warn", text))

    def info(self, text, **kwargs):
        self.lines.append(("info", text))


class FakeNode:
    # Parameter values to use instead of the node's defaults, as `ros2 run ... -p name:=value`
    # would. Set by reply(); empty means every parameter keeps its default.
    overrides = {}

    def __init__(self, name):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger

    def declare_parameter(self, name, default):
        return SimpleNamespace(value=self.overrides.get(name, default))

    def create_service(self, srv_type, name, callback):
        return None


class FakeBno055:
    """euler / gyro are either a value or an Exception instance to raise on access."""

    def __init__(self):
        self.euler_value = (10.0, 1.0, 2.0)
        self.gyro_value = (0.5, 0.1, -0.2)
        self.reads = []  # "euler" / "gyro", in the order the node read them (each read is one I2C transaction)

    @staticmethod
    def _get(value):
        if isinstance(value, Exception):
            raise value
        return value

    @property
    def euler(self):
        self.reads.append("euler")
        return self._get(self.euler_value)

    @property
    def gyro(self):
        self.reads.append("gyro")
        return self._get(self.gyro_value)


def install_stubs():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod

    module("rclpy")
    module("rclpy.node", Node=FakeNode)
    module("board", I2C=lambda: None)
    module("adafruit_icm20x", AccelRange=None, GyroRange=None)
    module("adafruit_bno055", BNO055_I2C=lambda i2c: FakeBno055())
    module("numpy")
    module("ahrs", Quaternion=None)
    module("ahrs.filters", Madgwick=None)
    module("ahrs.common")
    module("ahrs.common.orientation", acc2q=None)
    module("rocko_interfaces")
    module("rocko_interfaces.srv", Icm20948Data=None)


install_stubs()
spec = importlib.util.spec_from_file_location("ICM20948", NODE)
imu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu)


def reply(euler, gyro, read_pitch_rate=True):
    FakeNode.overrides = {"read_pitch_rate": read_pitch_rate}
    node = imu.ICM20948()
    node.icm.euler_value = euler
    node.icm.gyro_value = gyro
    response = SimpleNamespace(yaw=None, roll=None, pitch=None, pitch_rate_deg_s=None, success=False)
    return node.callback(None, response), node


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def test_default_does_not_read_the_gyro():
    # The real node is started without parameters (rocko.launch.py), so this is the robot's case.
    FakeNode.overrides = {}
    node = imu.ICM20948()
    check(node.read_pitch_rate is False, "read_pitch_rate must default to False")
    response = SimpleNamespace(yaw=None, roll=None, pitch=None, pitch_rate_deg_s=None, success=False)
    r = node.callback(None, response)
    check(node.icm.reads == ["euler"],
          f"with read_pitch_rate false the node must make ONE read (euler); it read {node.icm.reads}")
    check(r.success and r.pitch == 2.0 + node.pitch_offset, "the angles must be unaffected")
    check(math.isnan(r.pitch_rate_deg_s), "with read_pitch_rate false the rate must be NaN")
    check(not any(kind == "warn" for kind, _ in node.logger.lines),
          f"nothing is wrong, so nothing may be warned; logged {node.logger.lines}")


def test_good_angles_and_gyro():
    r, node = reply((10.0, 1.0, 2.0), (0.5, 0.1, -0.2))
    check(r.success, "a good read must succeed")
    check(r.pitch == 2.0 + node.pitch_offset, "pitch must include pitch_offset")
    check(math.isclose(r.pitch_rate_deg_s, math.degrees(0.5)), "rate = degrees(gyro[0]) by default")


def test_gyro_is_read_before_euler():
    # The gyro read is an extra I2C transaction. Done first, the pitch (euler) is the freshest sample
    # in the reply; done after, the pitch would age by the gyro read plus Python overhead.
    _, node = reply((10.0, 1.0, 2.0), (0.5, 0.1, -0.2))
    check(node.icm.reads == ["gyro", "euler"],
          f"the node must read the gyro first and euler once, last; it read {node.icm.reads}")


def test_bad_gyro_keeps_good_angles():
    for what, gyro in {"None": None, "None entry": (None, 0.1, 0.2), "NaN": (math.nan, 0.0, 0.0),
                       "inf": (math.inf, 0.0, 0.0), "exception": OSError("I2C error")}.items():
        r, node = reply((10.0, 1.0, 2.0), gyro)
        check(r.success, f"gyro {what}: the angles are good, so success must stay True")
        check(r.pitch == 2.0 + node.pitch_offset, f"gyro {what}: the good pitch must be sent")
        check(math.isnan(r.pitch_rate_deg_s), f"gyro {what}: the rate must be NaN")
        check(any("Pitch rate unavailable" in text for _, text in node.logger.lines),
              f"gyro {what}: the rate failure must be logged")


def test_out_of_range_rate_is_nan_and_logged():
    # Beyond the gyro's +/-2000 deg/s (MAX_ABS_PITCH_RATE_DEG_S) the node sends NaN and logs it
    # itself: the C++ plugin rejects such a rate too, but only warns after a first good one.
    check(imu.MAX_ABS_PITCH_RATE_DEG_S == 2000.0,
          "MAX_ABS_PITCH_RATE_DEG_S must equal kMaxAbsPitchRateDegS in ICM20948.hpp (2000 deg/s)")
    for gyro_x_rad_s in (40.0, -40.0):  # about +/-2292 deg/s
        r, node = reply((10.0, 1.0, 2.0), (gyro_x_rad_s, 0.0, 0.0))
        check(r.success, f"gyro {gyro_x_rad_s} rad/s: the angles are good, so success must stay True")
        check(math.isnan(r.pitch_rate_deg_s), f"gyro {gyro_x_rad_s} rad/s: the rate must be NaN")
        check(any("Pitch rate unavailable" in text for _, text in node.logger.lines),
              f"gyro {gyro_x_rad_s} rad/s: the out-of-range rate must be logged")
    r, _ = reply((10.0, 1.0, 2.0), (34.0, 0.0, 0.0))  # about 1948 deg/s: inside the range
    check(math.isclose(r.pitch_rate_deg_s, math.degrees(34.0)), "a rate inside the range must pass")


def test_still_gyro_is_a_real_reading():
    r, _ = reply((10.0, 1.0, 2.0), (0.0, 0.0, 0.0))
    check(r.success, "a gyro reading (0, 0, 0) must not fail the read")
    check(r.pitch_rate_deg_s == 0.0, "a still gyro is a rate of exactly 0.0, not NaN")


def test_bad_angles_fail_whatever_the_gyro():
    for what, euler in {"None": None, "None entry": (None, 1.0, 2.0),
                        "all identical": (0.0, 0.0, 0.0), "exception": OSError("I2C error")}.items():
        r, _ = reply(euler, (0.5, 0.1, -0.2))
        check(not r.success, f"euler {what}: success must be False")
        check(all(math.isnan(v) for v in (r.yaw, r.roll, r.pitch)), f"euler {what}: angles must be NaN")


def main():
    tests = [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as error:  # noqa: BLE001 - report every failure, not just the first
            failed += 1
            print(f"FAIL  {name}: {error!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
