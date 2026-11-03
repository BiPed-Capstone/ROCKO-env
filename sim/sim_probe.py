#!/usr/bin/env python3
"""Watch the simulated robot's pitch for a while, save what it saw, and say whether it passed.

Runs inside the dev container while rocko_sim.launch.py is up (verify_sim.sh starts it for you):

    /ros2_ws/src/ROCKO-env/sim/sim_probe.py --duration 8 --out /tmp/probe --mode report

It listens to /dynamic_joint_states (published by joint_state_broadcaster), and for every message
records the time, left_wheel_joint's pitch [deg] and its wheel velocity [m/s]. When it is done it
writes samples.csv and metrics.json into --out and prints a summary. metrics.json also holds
last_stamp_s, the stamp of the last message in seconds since the epoch.

Times are seconds since the first message, from the messages' own time stamps. "Move start" is the
last sample whose pitch still equals the very first sample's pitch exactly, just before the first
one that differs. SimPlant holds the robot perfectly still until the controllers start, so that is
(to within one 10 ms cycle) when the robot was released. The IMU delay shifts the start and the end
of a fall equally, so it cancels out of "time to exceed".

--mode decides the exit status (0 = PASS, 1 = FAIL, 2 = could not judge, e.g. no messages):
    report   PASS if any message arrived
    exceed   PASS if |pitch| reached --threshold-deg within --within seconds of move start
    change   PASS if the pitch moved at least --threshold-deg away from where it started
    still    PASS if the pitch stayed within --threshold-deg of where it started

These are numbers from a MODEL with ESTIMATED parameters. They say nothing about the real robot.
"""

import argparse
import csv
import json
import math
import os
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node

JOINT = "left_wheel_joint"  # both joints carry the same pitch (one IMU); either would do


class Probe(Node):
    def __init__(self):
        super().__init__("sim_probe")
        self.samples = []  # (time_s since the epoch of the stamps, pitch_deg, wheel_velocity_m_s)
        # Queue depth 100 = 1 s of the 100 Hz stream, so a slow moment in this script drops nothing.
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_message, 100)

    def on_message(self, msg):
        if JOINT not in msg.joint_names:
            return
        values = msg.interface_values[msg.joint_names.index(JOINT)]
        named = dict(zip(values.interface_names, values.values))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((stamp, named.get("pitch", math.nan), named.get("velocity", math.nan)))


def collect(duration_s, wait_s):
    """Spin until duration_s of messages have arrived, or wait_s passes with none at all."""
    rclpy.init()
    probe = Probe()
    started = time.monotonic()
    first_seen = None
    while rclpy.ok():
        rclpy.spin_once(probe, timeout_sec=0.05)
        now = time.monotonic()
        if probe.samples and first_seen is None:
            first_seen = now
        if first_seen is None and now - started > wait_s:
            break
        if first_seen is not None and now - first_seen >= duration_s:
            break
    samples = probe.samples
    probe.destroy_node()
    rclpy.shutdown()
    return samples


def metrics_of(samples, threshold_deg):
    t0 = samples[0][0]
    times = [s[0] - t0 for s in samples]
    pitch = [s[1] for s in samples]
    velocity = [s[2] for s in samples]
    start = pitch[0]
    finite_pitch = [p for p in pitch if math.isfinite(p)]
    finite_velocity = [v for v in velocity if math.isfinite(v)]

    # Move start: the last sample at exactly the starting pitch before the first that differs.
    move_start_s = None
    for i in range(1, len(pitch)):
        if pitch[i] != start:
            move_start_s = times[i - 1] if i >= 2 else None  # needs one unmoved sample after the first
            break
    exceed_s = next((t for t, p in zip(times, pitch) if abs(p) >= threshold_deg), None)
    return {
        "samples": len(samples),
        "nan_pitch_samples": len(pitch) - len(finite_pitch),
        "duration_s": times[-1],
        # The stamp of the LAST sample, in the messages' own clock (seconds since the epoch, the same
        # system clock as the [INFO] [<seconds>] stamps in the launch log). verify_sim.sh checks the
        # robot was released RELEASED_MIN_S before this, not before a time taken after the run.
        "last_stamp_s": samples[-1][0],
        "start_pitch_deg": start,
        "final_pitch_deg": pitch[-1],
        "max_abs_pitch_deg": max((abs(p) for p in finite_pitch), default=math.nan),
        "max_abs_delta_pitch_deg": max((abs(p - start) for p in finite_pitch), default=math.nan),
        "max_abs_wheel_velocity_m_s": max((abs(v) for v in finite_velocity), default=math.nan),
        "threshold_deg": threshold_deg,
        "move_start_s": move_start_s,
        "exceed_s": exceed_s,
        "time_to_exceed_s": (exceed_s - move_start_s
                             if exceed_s is not None and move_start_s is not None else None),
    }, times


def judge(mode, m, within_s):
    """Returns (exit status, one-line reason)."""
    if mode == "report":
        return 0, "data recorded"
    if mode == "change":
        moved = m["max_abs_delta_pitch_deg"]
        return (0 if moved >= m["threshold_deg"] else 1,
                f"pitch moved {moved:.4f} deg from its start (need >= {m['threshold_deg']})")
    if mode == "still":
        moved = m["max_abs_delta_pitch_deg"]
        return (0 if moved < m["threshold_deg"] else 1,
                f"pitch moved {moved:.4f} deg from its start (need < {m['threshold_deg']})")
    # mode == "exceed"
    if m["exceed_s"] is None:
        return 1, f"|pitch| never reached {m['threshold_deg']} deg"
    if m["time_to_exceed_s"] is None:
        return 2, "the probe did not see the pitch start to move, so the fall cannot be timed"
    late = m["time_to_exceed_s"] > within_s
    return (1 if late else 0,
            f"|pitch| reached {m['threshold_deg']} deg {m['time_to_exceed_s']:.3f} s after it "
            f"started to move (limit {within_s} s)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--duration", type=float, required=True, help="seconds of data to record")
    parser.add_argument("--out", required=True, help="directory for samples.csv and metrics.json")
    parser.add_argument("--mode", choices=["report", "exceed", "change", "still"], default="report")
    # 35 deg is the tipover angle of finding H-4 (TIPOVER_DEG in verify_sim.sh passes it explicitly).
    parser.add_argument("--threshold-deg", type=float, default=35.0,
                        help="pitch angle for exceed; size of the move for change/still [deg]")
    parser.add_argument("--within", type=float, default=math.inf,
                        help="exceed mode: time limit after move start [s]")
    parser.add_argument("--wait", type=float, default=60.0,
                        help="give up if no message arrives within this many seconds")
    args = parser.parse_args()

    samples = collect(args.duration, args.wait)
    if not samples:
        print(f"sim_probe: no /dynamic_joint_states message within {args.wait} s "
              "(is joint_state_broadcaster running?)")
        return 2

    m, times = metrics_of(samples, args.threshold_deg)
    status, reason = judge(args.mode, m, args.within)
    m.update({"mode": args.mode, "within_s": args.within, "exit_status": status, "reason": reason})

    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "samples.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "pitch_deg", "wheel_velocity_m_s"])
        for t, (_, p, v) in zip(times, samples):
            writer.writerow([f"{t:.4f}", repr(p), repr(v)])
    with open(os.path.join(args.out, "metrics.json"), "w") as f:
        json.dump(m, f, indent=2, default=str)

    print(f"sim_probe: {m['samples']} samples over {m['duration_s']:.2f} s; "
          f"start pitch {m['start_pitch_deg']:.4f} deg, final {m['final_pitch_deg']:.4f} deg, "
          f"max |pitch| {m['max_abs_pitch_deg']:.4f} deg, "
          f"max |change| {m['max_abs_delta_pitch_deg']:.4f} deg")
    exceed = (f"{m['time_to_exceed_s']:.3f} s after move start"
              if m["time_to_exceed_s"] is not None else "not timed")
    print(f"sim_probe: |pitch| >= {args.threshold_deg} deg: {exceed}")
    print(f"sim_probe: mode {args.mode}: {['PASS', 'FAIL', 'COULD NOT JUDGE'][status]}: {reason}")
    print(f"sim_probe: wrote {args.out}/samples.csv and metrics.json")
    return status


if __name__ == "__main__":
    sys.exit(main())
