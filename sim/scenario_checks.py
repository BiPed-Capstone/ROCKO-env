#!/usr/bin/env python3
"""Record what one of verify_sim.sh's stage 4 checks needs, judge it, and say why.

Runs inside the dev container while rocko_sim.launch.py is up; verify_sim.sh starts it for you.
Each subcommand is one check. It listens (and, for `teleop`, publishes), writes what it saw to
--out/metrics.json, prints a summary, and exits 0 = PASS, 1 = FAIL, 2 = could not judge.

    tipover     finding H-4. Watches the wheel COMMANDS the controllers write to the plant, from
                controller_manager's introspection topic. PASS if, from the first cycle the
                controllers write, every command written while |pitch| > --limit-deg is exactly
                0.0 on both wheels.
    teleop      finding C-3. Plays the gamepad: publishes TwistStamped on
                /diffbot_base_controller/cmd_vel at 20 Hz for 2 s, then stops. PASS if
                DiffDriveController's command_timed_out goes False while driven and True between
                --timeout-s and --timeout-s + --margin-s after the last command, the velocity
                reference was non-zero while driven and is exactly [0, 0] after the timeout, and
                /dynamic_joint_states keeps arriving (the plant is still being simulated).
    imu-hold    finding C-2. Watches the exported pitch through an injected IMU outage. PASS if,
                after the first finite pitch, no sample has pitch exactly 0.0 or NaN.
    pitch-rate  finding H-6. PASS if pitch_pid_controller's feedback_dot (the derivative its D term
                uses) is finite, clearly non-zero, equal to the plant's exported pitch_rate in the
                same control cycle, and error_dot == -feedback_dot.

The thresholds are passed in by verify_sim.sh, which says why each one is what it is. These are
results about a MODEL with ESTIMATED parameters. They say nothing about the real robot.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from control_msgs.msg import DynamicJointState, MultiDOFCommand, MultiDOFStateStamped
from geometry_msgs.msg import TwistStamped
from pal_statistics_msgs.msg import Statistics
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

JOINTS = ("left_wheel_joint", "right_wheel_joint")
PASS, FAIL, COULD_NOT_JUDGE = 0, 1, 2
# Queue depth 200 = 2 s of a 100 Hz stream, so a slow moment in this script drops nothing.
DEPTH = 200


class Recorder(Node):
    """Subscribes to the topics it is given and keeps every message with the time it ARRIVED
    (time.monotonic(), seconds; the same clock this script uses for anything it publishes)."""

    def __init__(self, subscriptions):
        super().__init__("verify_sim_scenario_checks")
        self.received = {}
        for topic, msg_type, qos in subscriptions:
            self.received[topic] = []
            self.create_subscription(
                msg_type, topic, lambda msg, t=topic: self.received[t].append((time.monotonic(), msg)),
                qos)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, end - time.monotonic())))

    def spin_until(self, condition, timeout_s):
        """Spin until condition() is true; returns False if timeout_s passes first."""
        end = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.01)
        return True


def stamp_s(msg):
    """A message's header stamp in seconds since the epoch (the controller_manager's clock)."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def joint_values(msg, joint):
    """{interface name: value} for one joint of a DynamicJointState message."""
    if joint not in msg.joint_names:
        return {}
    values = msg.interface_values[msg.joint_names.index(joint)]
    return dict(zip(values.interface_names, values.values))


def finish(out_dir, metrics, status, reason):
    metrics.update({"exit_status": status, "reason": reason})
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2, default=str)
    print(f"scenario_checks: {['PASS', 'FAIL', 'COULD NOT JUDGE'][status]}: {reason}")
    return status


# ---- H-4: tipover cutoff -----------------------------------------------------------------------
def check_tipover(args):
    # The controller_manager publishes every state and command interface's value, all from the
    # same moment, on this topic (it is how ros2_control lets you look inside the loop). The
    # command interface is exactly what AddFeedforwardController wrote for the plant.
    topic = "/controller_manager/introspection_data/full"
    names = {
        "pitch": "state_interface.left_wheel_joint/pitch",
        "left": "command_interface.left_wheel_joint/velocity",
        "right": "command_interface.right_wheel_joint/velocity",
    }
    # It is published best-effort, so the subscription must be best-effort too or it gets nothing.
    best_effort = QoSProfile(depth=DEPTH, reliability=ReliabilityPolicy.BEST_EFFORT)
    rec = Recorder([(topic, Statistics, best_effort)])
    samples = []  # (stamp_s, pitch_deg, left_command_rad_s, right_command_rad_s)

    def collect_new():
        while rec.received[topic]:
            _, msg = rec.received[topic].pop(0)
            named = {s.name: s.value for s in msg.statistics}
            if all(n in named for n in names.values()):
                samples.append((stamp_s(msg), named[names["pitch"]], named[names["left"]],
                                named[names["right"]]))

    def writing():
        """True once the controllers have written a command (before that the slots are NaN)."""
        collect_new()
        return any(math.isfinite(s[2]) and math.isfinite(s[3]) for s in samples)

    if not rec.spin_until(writing, args.wait):
        return finish(args.out, {"samples": len(samples)}, COULD_NOT_JUDGE,
                      f"the controllers wrote no wheel command within {args.wait} s")
    rec.spin_for(args.record)
    collect_new()
    rec.destroy_node()

    written = [s for s in samples if math.isfinite(s[2]) and math.isfinite(s[3])]
    past = [s for s in written if abs(s[1]) > args.limit_deg]
    nonzero = [s for s in past if s[2] != 0.0 or s[3] != 0.0]
    metrics = {
        "samples": len(samples),
        "samples_with_commands": len(written),
        "samples_past_limit_with_commands": len(past),
        "nonzero_commands_past_limit": len(nonzero),
        "first_written_sample": written[0],
        "max_abs_command_rad_s": max(max(abs(s[2]), abs(s[3])) for s in written),
        "limit_deg": args.limit_deg,
        "last_stamp_s": samples[-1][0],
    }
    print(f"scenario_checks: {len(written)} samples with wheel commands, {len(past)} of them with "
          f"|pitch| > {args.limit_deg} deg; first: pitch {written[0][1]:.2f} deg, commands "
          f"{written[0][2]!r}, {written[0][3]!r} rad/s; max |command| "
          f"{metrics['max_abs_command_rad_s']:.4g} rad/s")
    if len(past) < args.min_samples:
        return finish(args.out, metrics, COULD_NOT_JUDGE,
                      f"only {len(past)} samples past the limit (need {args.min_samples}): the "
                      "scenario did not hold the robot past the limit")
    if nonzero:
        s = nonzero[0]
        return finish(args.out, metrics, FAIL,
                      f"{len(nonzero)} of {len(past)} samples past the limit had a non-zero wheel "
                      f"command; first: pitch {s[1]:.2f} deg, commands {s[2]:.4g}, {s[3]:.4g} rad/s")
    return finish(args.out, metrics, PASS,
                  f"all {len(past)} wheel commands written with |pitch| > {args.limit_deg} deg "
                  "were exactly 0.0 on both wheels")


# ---- C-3: teleop watchdog ----------------------------------------------------------------------
def check_teleop(args):
    flag_topic = "/diff_drive_controller/command_timed_out"
    reference_topic = "/velocity_pid_controller/reference"
    plant_topic = "/dynamic_joint_states"
    # transient_local, as the publisher: the current state arrives even though we start late.
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rec = Recorder([(flag_topic, Bool, latched), (reference_topic, MultiDOFCommand, DEPTH),
                    (plant_topic, DynamicJointState, DEPTH)])
    gamepad = rec.create_publisher(TwistStamped, "/diffbot_base_controller/cmd_vel", 10)

    # Wait for DiffDriveController: its flag has arrived and it subscribes to the gamepad topic.
    ready = rec.spin_until(
        lambda: rec.received[flag_topic] and gamepad.get_subscription_count() > 0, args.wait)
    if not ready:
        rec.destroy_node()
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"DiffDriveController did not appear within {args.wait} s")
    rec.spin_for(0.5)  # half a second with no gamepad: the flag should say timed out

    # Play the gamepad: the same command every 1/rate s, for drive_s seconds.
    command = TwistStamped()
    command.twist.linear.x = args.speed_m_s
    publish_times = []
    next_publish = time.monotonic()
    for _ in range(round(args.drive_s * args.rate_hz)):
        rec.spin_until(lambda: time.monotonic() >= next_publish, 1.0)
        command.header.stamp = rec.get_clock().now().to_msg()
        gamepad.publish(command)
        publish_times.append(time.monotonic())
        next_publish += 1.0 / args.rate_hz
    rec.spin_for(args.after_s)  # ... and then the gamepad goes silent
    rec.destroy_node()

    first_pub, last_pub = publish_times[0], publish_times[-1]
    flags = rec.received[flag_topic]  # (arrival time, Bool)
    references = rec.received[reference_topic]
    plant = rec.received[plant_topic]
    settled = last_pub + args.timeout_s + args.margin_s  # by now the reference must be zero

    went_false = [t for t, m in flags if not m.data and first_pub <= t <= last_pub]
    true_while_driven = [t for t, m in flags
                         if m.data and went_false and went_false[0] < t <= last_pub]
    went_true = [t for t, m in flags if m.data and t > last_pub]
    latency = went_true[0] - last_pub if went_true else None
    driven_values = [list(m.values) for t, m in references if first_pub < t <= last_pub]
    after_values = [list(m.values) for t, m in references if t > settled]
    plant_after = [t for t, _ in plant if t > settled]
    metrics = {
        "commands_published": len(publish_times),
        "drive_s": last_pub - first_pub,
        "flag_messages": [(round(t - first_pub, 4), m.data) for t, m in flags],
        "timeout_latency_s": latency,
        "timeout_s": args.timeout_s,
        "margin_s": args.margin_s,
        "references_while_driven": len(driven_values),
        "references_while_driven_equal_to_command": sum(
            v == [args.speed_m_s] * 2 for v in driven_values),
        "references_after_timeout": len(after_values),
        "nonzero_references_after_timeout": sum(v != [0.0, 0.0] for v in after_values),
        "plant_samples_after_timeout": len(plant_after),
        "after_window_s": last_pub + args.after_s - settled,
    }
    print(f"scenario_checks: {len(publish_times)} gamepad commands over {metrics['drive_s']:.2f} s; "
          f"flag (s after first command, value): {metrics['flag_messages']}")
    print(f"scenario_checks: timed out {latency if latency is None else round(latency, 4)} s after "
          f"the last command (need {args.timeout_s:g} to {args.timeout_s + args.margin_s:g} s); "
          f"references while driven {len(driven_values)} "
          f"({metrics['references_while_driven_equal_to_command']} equal to the command), after "
          f"the timeout {len(after_values)} ({metrics['nonzero_references_after_timeout']} "
          f"non-zero); plant samples after the timeout {len(plant_after)}")

    if not went_false:
        return finish(args.out, metrics, FAIL, "command_timed_out never went False while driven")
    if true_while_driven:
        return finish(args.out, metrics, FAIL,
                      "command_timed_out went True while commands were still arriving")
    if latency is None:
        return finish(args.out, metrics, FAIL,
                      f"command_timed_out never went True in the {args.after_s} s after the last "
                      "command")
    # Never earlier than the timeout (less one 10 ms timer period of rounding), never later than
    # timeout + margin.
    if not (args.timeout_s - 0.01 <= latency <= args.timeout_s + args.margin_s):
        return finish(args.out, metrics, FAIL,
                      f"command_timed_out went True {latency:.3f} s after the last command, outside "
                      f"[{args.timeout_s:g}, {args.timeout_s + args.margin_s:g}] s")
    if metrics["references_while_driven_equal_to_command"] == 0:
        return finish(args.out, metrics, FAIL,
                      f"no velocity reference equal to the command [{args.speed_m_s}] * 2 while "
                      "driven, so a zero afterwards would prove nothing")
    if len(after_values) < args.min_samples or metrics["nonzero_references_after_timeout"]:
        return finish(args.out, metrics, FAIL,
                      f"after the timeout: {len(after_values)} references (need >= "
                      f"{args.min_samples}), {metrics['nonzero_references_after_timeout']} non-zero")
    if len(plant_after) < args.min_samples:
        return finish(args.out, metrics, FAIL,
                      f"only {len(plant_after)} /dynamic_joint_states messages after the timeout "
                      f"(need >= {args.min_samples}): the plant stopped being simulated")
    return finish(args.out, metrics, PASS,
                  f"timed out {latency:.3f} s after the last command, the velocity reference went "
                  f"from [{args.speed_m_s}] * 2 to exactly [0, 0] ({len(after_values)} messages), "
                  f"and {len(plant_after)} plant samples arrived after the timeout")


# ---- C-2: the exported pitch during an IMU outage -----------------------------------------------
def check_imu_hold(args):
    topic = "/dynamic_joint_states"
    rec = Recorder([(topic, DynamicJointState, DEPTH)])
    samples = []  # (stamp_s, left pitch_deg, right pitch_deg)

    def collect_new():
        while rec.received[topic]:
            _, msg = rec.received[topic].pop(0)
            left, right = joint_values(msg, JOINTS[0]), joint_values(msg, JOINTS[1])
            samples.append((stamp_s(msg), left.get("pitch", math.nan), right.get("pitch", math.nan)))

    def released():
        """True once the pitch has moved from its first value (SimPlant holds it still until the
        controllers command the wheels; see sim_probe.py)."""
        collect_new()
        return len(samples) > 1 and samples[-1][1] != samples[0][1]

    if not rec.spin_until(released, args.wait):
        rec.destroy_node()
        return finish(args.out, {"samples": len(samples)}, COULD_NOT_JUDGE,
                      f"the simulated pitch did not start moving within {args.wait} s")
    rec.spin_for(args.record)  # long enough to cover the whole outage
    collect_new()
    rec.destroy_node()

    move_start = next(s[0] for s in samples if s[1] != samples[0][1])
    first_finite = next((i for i, s in enumerate(samples)
                         if math.isfinite(s[1]) and math.isfinite(s[2])), None)
    after = samples[first_finite:] if first_finite is not None else []
    bad = [s for s in after if not (math.isfinite(s[1]) and math.isfinite(s[2]))
           or s[1] == 0.0 or s[2] == 0.0]
    # The longest run of identical consecutive pitch values after the robot started moving: a held
    # value during the outage shows up as a run (information only; a dropped message shortens it).
    longest, run = 1, 1
    moving = [s[1] for s in samples if s[0] >= move_start]
    for a, b in zip(moving, moving[1:]):
        run = run + 1 if a == b else 1
        longest = max(longest, run)
    metrics = {
        "samples": len(samples),
        "samples_after_first_finite": len(after),
        "zero_or_nan_after_first_finite": len(bad),
        "first_bad": bad[0] if bad else None,
        "last_sample_s_after_move_start": samples[-1][0] - move_start,
        "longest_identical_run_after_move_start": longest,
        # verify_sim.sh compares this with SimPlant's release stamp, to make sure the recording
        # reached the outage. (Not "move start" here: the exported pitch starts to change one IMU
        # delay, ~20 ms, after the release, which is what the outage time counts from.)
        "last_stamp_s": samples[-1][0],
    }
    print(f"scenario_checks: {len(samples)} samples, the last {metrics['last_sample_s_after_move_start']:.3f}"
          f" s after the pitch started moving; longest run of one held pitch value after that: "
          f"{longest} samples; pitch 0.0 or NaN after the first good read: {len(bad)}")
    if bad:
        return finish(args.out, metrics, FAIL,
                      f"{len(bad)} samples exported pitch 0.0 or NaN after the first good read; "
                      f"first: {bad[0]}")
    return finish(args.out, metrics, PASS,
                  f"no pitch of exactly 0.0 and no NaN in {len(after)} samples after the first "
                  "good read")


# ---- H-6: the D term's derivative comes from the gyro rate -------------------------------------
def check_pitch_rate(args):
    state_topic = "/pitch_pid_controller/controller_state"
    plant_topic = "/dynamic_joint_states"
    rec = Recorder([(state_topic, MultiDOFStateStamped, DEPTH),
                    (plant_topic, DynamicJointState, DEPTH)])
    if not rec.spin_until(lambda: rec.received[state_topic], args.wait):
        rec.destroy_node()
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no {state_topic} message within {args.wait} s")
    rec.spin_for(args.record)
    rec.destroy_node()

    plant = [(stamp_s(m), m) for _, m in rec.received[plant_topic]]
    pairs = []  # (feedback_dot, error_dot, exported pitch_rate), deg/s, one per joint per cycle
    for _, state in rec.received[state_topic]:
        # The same control cycle: the plant message stamped nearest, within half a 10 ms cycle
        # less a margin (the two are stamped about 0.1 ms apart within one cycle).
        t = stamp_s(state)
        nearest = min(plant, key=lambda p: abs(p[0] - t), default=None)
        if nearest is None or abs(nearest[0] - t) > args.pair_window_s:
            continue
        for dof in state.dof_states:
            rate = joint_values(nearest[1], dof.name).get("pitch_rate", math.nan)
            pairs.append((dof.feedback_dot, dof.error_dot, rate))

    finite = all(math.isfinite(v) for p in pairs for v in p)
    max_rate = max((abs(p[0]) for p in pairs), default=math.nan)
    max_mismatch = max((abs(p[0] - p[2]) for p in pairs), default=math.nan)
    max_error_dot_sum = max((abs(p[1] + p[0]) for p in pairs), default=math.nan)
    metrics = {
        "controller_state_messages": len(rec.received[state_topic]),
        "pairs": len(pairs),
        "all_finite": finite,
        "max_abs_feedback_dot_deg_s": max_rate,
        "max_abs_feedback_dot_minus_pitch_rate_deg_s": max_mismatch,
        "max_abs_error_dot_plus_feedback_dot_deg_s": max_error_dot_sum,
    }
    print(f"scenario_checks: {len(pairs)} (feedback_dot, pitch_rate) pairs; max |feedback_dot| "
          f"{max_rate:.4g} deg/s; max |feedback_dot - pitch_rate| {max_mismatch:.3g}; "
          f"max |error_dot + feedback_dot| {max_error_dot_sum:.3g}")
    if len(pairs) < args.min_samples:
        return finish(args.out, metrics, COULD_NOT_JUDGE,
                      f"only {len(pairs)} pairs from the same cycle (need {args.min_samples})")
    if not finite:
        return finish(args.out, metrics, FAIL, "a feedback_dot, error_dot or pitch_rate was not finite")
    if not max_rate >= args.min_rate_deg_s:
        return finish(args.out, metrics, FAIL,
                      f"|feedback_dot| never reached {args.min_rate_deg_s} deg/s: the D term is not "
                      "reading a rate")
    if not max_mismatch <= args.tolerance_deg_s:
        return finish(args.out, metrics, FAIL,
                      f"feedback_dot differs from the exported pitch_rate by up to {max_mismatch:.3g}"
                      f" deg/s (tolerance {args.tolerance_deg_s})")
    if not max_error_dot_sum <= args.tolerance_deg_s:
        return finish(args.out, metrics, FAIL,
                      f"error_dot is not -feedback_dot (off by up to {max_error_dot_sum:.3g} deg/s): "
                      "the rate reference is not 0")
    return finish(args.out, metrics, PASS,
                  f"feedback_dot equals the exported pitch_rate in all {len(pairs)} pairs (max "
                  f"|feedback_dot| {max_rate:.3g} deg/s) and error_dot == -feedback_dot")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="check", required=True)

    def add(name, function, record_help):
        p = sub.add_parser(name)
        p.add_argument("--out", required=True, help="directory for metrics.json")
        p.add_argument("--wait", type=float, default=60.0,
                       help="give up (could not judge) if nothing arrives within this many seconds")
        if record_help:
            p.add_argument("--record", type=float, default=3.0, help=record_help)
        # 50 samples = 0.5 s of a 100 Hz stream. verify_sim.sh passes its MIN_SAMPLES explicitly.
        p.add_argument("--min-samples", type=int, default=50,
                       help="fewer samples than this to judge from = could not judge / fail")
        p.set_defaults(function=function)
        return p

    p = add("tipover", check_tipover, "seconds to record after the first written command")
    p.add_argument("--limit-deg", type=float, required=True, help="tipover_pitch_deg [deg]")
    p = add("teleop", check_teleop, None)  # its timing is --drive-s and --after-s
    p.add_argument("--timeout-s", type=float, required=True, help="the node's command_timeout_s")
    p.add_argument("--margin-s", type=float, required=True, help="allowed lateness [s]")
    p.add_argument("--speed-m-s", type=float, required=True, help="linear.x of the fake gamepad")
    p.add_argument("--rate-hz", type=float, default=20.0, help="fake gamepad rate [Hz]")
    p.add_argument("--drive-s", type=float, default=2.0, help="how long the fake gamepad drives [s]")
    p.add_argument("--after-s", type=float, default=2.0, help="how long to listen after it stops [s]")
    add("imu-hold", check_imu_hold, "seconds to record after the pitch starts moving")
    p = add("pitch-rate", check_pitch_rate, "seconds to record after the first controller_state")
    p.add_argument("--min-rate-deg-s", type=float, required=True,
                   help="|feedback_dot| must reach this at least once [deg/s]")
    p.add_argument("--tolerance-deg-s", type=float, required=True,
                   help="allowed |feedback_dot - pitch_rate| and |error_dot + feedback_dot| [deg/s]")
    p.add_argument("--pair-window-s", type=float, default=0.003,
                   help="max stamp difference for two messages to count as one cycle [s]")
    args = parser.parse_args()

    rclpy.init()
    try:
        return args.function(args)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
