#!/usr/bin/env python3
"""Run the real BatteryMonitor node against a fake INA226 and check what it publishes. Needs ROS: the container.

    docker compose -f docker/compose.yml exec -T ros2 bash -c \
      'source /opt/ros/jazzy/setup.bash && python3 /ros2_ws/src/ROCKO-env/tools/battery_monitor_check.py'

The container has no INA226, so the node's Ina226 class is swapped for a fake that answers like one (13.000 V), and
can be told to fail. Checks:
  1. every reading goes out on /rocko/battery_raw, at roughly the sample rate, unsmoothed, with a header stamp;
  2. /rocko/battery (smoothed) still comes once a second;
  3. one failed and one implausible reading are skipped and counted, and the monitor stays OK;
  4. a second with no good reading reports WARN and publishes no smoothed voltage;
  5. readings that keep failing for battery.GIVE_UP_AFTER_S turn it to ERROR, with the cause;
  6. a chip that answers "TI" but is not an INA226 (another die ID) is refused.
The rate check is loose on purpose (at least RATE_FLOOR of the nominal count): a Python timer in a busy container is
not the Pi, and this check is about "every reading is published", not timing. Nothing here says what 50 Hz costs on
the Pi, or that the real chip and bus keep up.
"""
import importlib.util
import pathlib
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "rocko_env"))
from rocko_env.extras import battery, health  # noqa: E402

spec = importlib.util.spec_from_file_location("BatteryMonitor", REPO / "rocko_env/rocko_env/extras/BatteryMonitor.py")
monitor_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor_module)   # main() is behind `if __name__ == "__main__"`, so nothing runs

GOOD_COUNTS = 10400        # REG_BUS_VOLTAGE counts for 13.000 V (1.25 mV per count)
RUN_S = 3.0                # [s] how long the first part listens
RATE_FLOOR = 0.6           # the share of the nominal reading count that must arrive


class FakeIna226:
    """Answers like an INA226 at 13.000 V. `fail_next` makes the next reads raise; `zero_next` makes them read 0 V;
    `always_fail` makes every read raise."""
    fail_next = 0
    zero_next = 0
    always_fail = False
    die_id = battery.DIE_ID_INA226

    def __init__(self, device, address):
        pass

    def read_u16(self, register):
        if register == battery.REG_MANUFACTURER_ID:
            return battery.MANUFACTURER_TI
        if register == battery.REG_DIE_ID:
            return FakeIna226.die_id
        if FakeIna226.always_fail or FakeIna226.fail_next > 0:
            FakeIna226.fail_next = max(0, FakeIna226.fail_next - 1)
            raise OSError(121, "Remote I/O error")
        if FakeIna226.zero_next > 0:
            FakeIna226.zero_next -= 1
            return 0
        return GOOD_COUNTS

    def write_u16(self, register, value):
        pass

    def close(self):
        pass


failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(name)


def main():
    monitor_module.Ina226 = FakeIna226
    rclpy.init()
    monitor = monitor_module.BatteryMonitor()
    listener = Node("battery_monitor_check")
    raw, smoothed, reports = [], [], []
    listener.create_subscription(BatteryState, "/rocko/battery_raw", raw.append, 100)
    listener.create_subscription(BatteryState, "/rocko/battery", smoothed.append, 10)

    def on_diagnostics(msg):
        found = health.from_message(msg)
        if "battery" in found:
            reports.append(found["battery"])
    listener.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(listener)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)

    spin_for(0.5)                      # discovery
    raw.clear()
    smoothed.clear()
    spin_for(RUN_S)
    expected = battery.DEFAULT_SAMPLE_RATE_HZ * RUN_S
    check(f"raw readings published at about {battery.DEFAULT_SAMPLE_RATE_HZ:.0f} Hz",
          len(raw) >= RATE_FLOOR * expected, f"{len(raw)} in {RUN_S:.0f} s, nominal {expected:.0f}")
    check("raw readings are the unsmoothed value, 13.000 V", raw and all(abs(m.voltage - 13.0) < 1e-6 for m in raw))
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in raw]
    check("raw readings carry a header stamp that rises", len(stamps) > 1 and stamps[0] > 0
          and all(b > a for a, b in zip(stamps, stamps[1:])))
    check("the smoothed voltage still comes once a second", 2 <= len(smoothed) <= 4, f"{len(smoothed)} in {RUN_S:.0f} s")

    FakeIna226.fail_next, FakeIna226.zero_next = 1, 1
    reports.clear()
    spin_for(1.5)
    last = reports[-1] if reports else None
    check("one failed and one implausible reading: still OK", last is not None and last["level"] == health.OK,
          last and last["message"])
    check("the two skipped readings are counted", last is not None and last["values"].get("skipped_readings") == "2",
          last and last["values"])
    check("the measured sample rate is reported", last is not None and float(last["values"].get("sample_rate_hz", 0)) > 0,
          last and last["values"].get("sample_rate_hz"))

    # Called directly, not waited for: when a report falls inside a run of failures depends on timing.
    smoothed.clear()
    monitor.good_since_report = 0
    monitor.report()
    spin_for(0.2)
    check("a second with no good reading: WARN, and no smoothed voltage published",
          monitor.status["level"] == health.WARN and "no good reading" in monitor.status["message"] and not smoothed,
          monitor.status["message"])

    FakeIna226.always_fail = True
    reports.clear()
    spin_for(battery.GIVE_UP_AFTER_S + 1.5)
    last = reports[-1] if reports else None
    check(f"readings failing for {battery.GIVE_UP_AFTER_S:.0f} s: ERROR, with the cause",
          last is not None and last["level"] == health.ERROR and "read failed" in last["message"],
          last and last["message"])

    FakeIna226.always_fail = False
    FakeIna226.die_id = 0x2281          # what an INA228 answers: a TI chip, but not an INA226
    monitor.open_sensor()
    check("a TI chip that is not an INA226 is refused", monitor.sensor is None
          and "not an INA226" in monitor.status["message"], monitor.status["message"])

    executor.shutdown()
    monitor.destroy_node()
    listener.destroy_node()
    rclpy.shutdown()
    print(f"RESULT: {'FAIL (' + str(len(failures)) + ' failed)' if failures else 'PASS'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
