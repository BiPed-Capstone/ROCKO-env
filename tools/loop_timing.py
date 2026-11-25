#!/usr/bin/env python3
"""Measure the control loop's timing on the robot from controller_manager's own statistics.

Run ON THE PI, while a stack is running (as root, through the ROCKO environment):

    pi/rocko_run.sh python3 tools/loop_timing.py --seconds 60 --label balance-minimal

controller_manager (ros2_control 4.x) publishes, every control cycle, how long each part of the cycle
took and how often each component and controller actually ran, on
/controller_manager/statistics/names and /controller_manager/statistics/values. This script listens
for --seconds, then prints, per quantity, the median, the 99th percentile and the largest value seen:

  - controller_manager.read_time / update_time / write_time / total_time [us]: how long the whole
    read(), update() and write() of one cycle took. The budget is the cycle (10 ms at 100 Hz).
  - <name> period [ms]: 1 / that component's or controller's measured rate, per cycle. Its spread is the
    loop's jitter: how far the real cycle length strays from the nominal one.
  - <name> exec [us]: how long that one component's read()/write() or that controller's update() took.

What it cannot show: anything between the samples it lost (the values topic is best-effort, so the
count it received is printed next to the count expected); a cycle that never started (the rates are
measured by controller_manager itself, at each cycle that ran); and the cost of anything outside
controller_manager except through its effect on these numbers. It adds load of its own to the Pi it
measures (one subscriber taking 100 messages a second). Numbers from the container or the mock say
nothing about the Pi (CLAUDE.md section 5). The quantity names and units above were checked against a
real /controller_manager/statistics/names message on the robot (2026-11-24).
"""

import argparse
import math
import sys
import time


def summarise(samples):
    """(count, median, 99th percentile, max) of a list of numbers; NaNs are dropped. Nearest-rank
    percentiles: the value at rank ceil(q * n), so with 100 samples the 99th percentile is the 99th
    smallest and the median is the 50th (the lower of the two middle values for an even count).
    (None, None, None) for the statistics when nothing is left."""
    values = sorted(v for v in samples if not math.isnan(v))
    n = len(values)
    if n == 0:
        return 0, None, None, None

    def rank(q):
        return values[max(0, math.ceil(q * n) - 1)]

    return n, rank(0.5), rank(0.99), values[-1]


def owner_of(name):
    """'Gyro.stats/read_cycle/periodicity/current_value' -> 'Gyro read'; controllers have no read/write."""
    owner = name.split(".stats/")[0]
    if "/read_cycle/" in name:
        return owner + " read"
    if "/write_cycle/" in name:
        return owner + " write"
    return owner


def cycle_series(names, values_messages):
    """Turn a list of value lists (one per cycle, in the order of `names`) into {quantity: [per-cycle]}.
    Periodicity [Hz] becomes a period [ms]; a rate of 0 or NaN becomes NaN."""
    series = {}
    for values in values_messages:
        for name, value in zip(names, values):
            if name.startswith("controller_manager.") and name.endswith("_time"):
                series.setdefault(name + " [us]", []).append(value)
            elif name.endswith("/periodicity/current_value"):
                period_ms = 1000.0 / value if value and not math.isnan(value) else math.nan
                series.setdefault(owner_of(name) + " period [ms]", []).append(period_ms)
            elif name.endswith("/execution_time/current_value"):
                series.setdefault(owner_of(name) + " exec [us]", []).append(value)
    return series


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--rate-hz", type=float, default=100.0,
                        help="update_rate in the mode's controllers YAML (100 balance, 50 tank); used only to "
                             "print how many samples to expect")
    parser.add_argument("--label", default="", help="a name for this run, printed in the header")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from pal_statistics_msgs.msg import StatisticsNames, StatisticsValues
    except ImportError:
        print("ROS is not set up in this shell: run it through pi/rocko_run.sh (see the top of this file)",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("loop_timing")
    names = {}          # names_version -> list of names
    received = []       # (names_version, values) per cycle
    names_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    values_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)

    def on_names(msg):
        names[msg.names_version] = list(msg.names)

    def on_values(msg):
        received.append((msg.names_version, list(msg.values)))

    node.create_subscription(StatisticsNames, "/controller_manager/statistics/names", on_names, names_qos)
    node.create_subscription(StatisticsValues, "/controller_manager/statistics/values", on_values, values_qos)

    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not names:
        print("no statistics names arrived: is a stack running, with controller_manager up, on the same "
              "ROS_DOMAIN_ID (run through pi/rocko_run.sh)?", file=sys.stderr)
        return 1
    if not received:
        print("statistics names arrived but no values", file=sys.stderr)
        return 1
    version = max(names)  # the current set of names; samples from an older set are dropped
    cycles = [v for ver, v in received if ver == version]
    if not cycles:
        print(f"{len(received)} samples arrived, none for the current names (version {version})", file=sys.stderr)
        return 1
    series = cycle_series(names[version], cycles)

    print(f"loop timing {args.label}: {len(received)} cycles received, {len(cycles)} used, in {args.seconds:.0f} s "
          f"(about {args.seconds * args.rate_hz:.0f} expected at {args.rate_hz:.0f} Hz)")
    print(f"{'quantity':52s} {'n':>6s} {'median':>9s} {'p99':>9s} {'max':>9s}")
    omitted = []
    for quantity in sorted(series):
        n, median, p99, biggest = summarise(series[quantity])
        if n == 0 or biggest == 0.0:
            omitted.append(quantity)  # all NaN, or stayed 0 (e.g. the controller-switch times while none happens)
            continue
        print(f"{quantity:52s} {n:6d} {median:9.3f} {p99:9.3f} {biggest:9.3f}")
    if omitted:
        print("omitted (all 0 or NaN): " + ", ".join(omitted))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
