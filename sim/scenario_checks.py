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

The three balance scenarios (`verify_sim.sh scenarios`, bootstrap-sim step 7). Each records
/dynamic_joint_states from the moment SimPlant releases the robot (the stamp of its "releasing the
simulated robot" line in --launch-log; t = 0 there, the same zero SimPlant's own impulse time uses)
and writes samples.csv as well as metrics.json:
    upright        U. PASS if, for --record s after release, |pitch| never reaches --tipover-deg,
                   and from --settle-from s to the end |pitch| <= --settled-pitch-deg and the mean
                   measured wheel speed |v| <= --max-speed-m-s. verify_sim.sh also uses it for S2b
                   and AB, judging metrics.json's fell_at_s itself.
    impulse        I. PASS if the push SimPlant applies at --impulse-time-s shows as a jump of at
                   least 80 % of --impulse-deg-s in the exported pitch_rate, |pitch| never reaches
                   --tipover-deg, and within --settle-within-s of the push |pitch| is back under
                   --settled-pitch-deg and stays there to the end of --record.
    velocity-step  V. Plays the gamepad (as `teleop` does) through the real DiffDriveController:
                   linear.x = 0 for --before-s, then a step to --speed-m-s held for --step-s. PASS if
                   the mean measured wheel velocity reaches --rise-fraction of the step within
                   --rise-within-s, overshoots it by at most --max-overshoot-pct, |pitch| stays
                   under --max-pitch-deg, and never reaches --tipover-deg.
"Measured wheel velocity" is the exported `velocity` state [m/s] the velocity loop itself reads (the
mean of the two wheels). With model_encoder_node it is about HALF the true speed (finding U-2), and
its sign is encoder_sign times the direction a positive command drives the robot (sim-plant.md §2.4).

The thresholds are passed in by verify_sim.sh, which says why each one is what it is. These are
results about a MODEL with ESTIMATED parameters. They say nothing about the real robot.
"""

import argparse
import csv
import json
import math
import os
import re
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


# ---- The balance scenarios: U, I, V (bootstrap-sim step 7) -------------------------------------
# SimPlant's log line when it releases the robot, e.g.
#   "[ros2_control_node-1] [INFO] [1793667068.42] [SimRocko]: SimPlant: ... releasing the simulated robot"
RELEASE_LINE = re.compile(r"\[INFO\] \[(\d+\.\d+)\].*releasing the simulated robot")


def release_stamp(launch_log):
    """The epoch-seconds stamp of SimPlant's release line, or None if it is not in the log yet."""
    try:
        with open(launch_log, errors="replace") as f:
            for line in f:
                m = RELEASE_LINE.search(line)
                if m:
                    return float(m.group(1))
    except OSError:
        pass
    return None


def record_from_release(rec, args, topic="/dynamic_joint_states", during=None):
    """Spin until --record s of plant samples after the release have arrived. Returns (release
    stamp or None, samples), samples = [(t_s after release, pitch_deg, pitch_rate_deg_s,
    mean measured wheel velocity m/s)]; pitch and rate are left_wheel_joint's (one IMU feeds both
    joints). during(t_s) is called while spinning with the latest sample's time (for publishing)."""
    deadline = time.monotonic() + args.wait
    release = None
    while release is None and time.monotonic() < deadline:
        rec.spin_for(0.1)
        release = release_stamp(args.launch_log)
        if during:
            during(None)
    if release is None:
        return None, []
    samples = []

    def collect_new():
        while rec.received[topic]:
            _, msg = rec.received[topic].pop(0)
            left, right = joint_values(msg, JOINTS[0]), joint_values(msg, JOINTS[1])
            v = 0.5 * (left.get("velocity", math.nan) + right.get("velocity", math.nan))
            samples.append((stamp_s(msg) - release, left.get("pitch", math.nan),
                            left.get("pitch_rate", math.nan), v))

    end = time.monotonic() + args.record + args.wait
    while time.monotonic() < end:
        rec.spin_for(0.02)
        collect_new()
        if during:
            during(samples[-1][0] if samples else None)
        if samples and samples[-1][0] >= args.record:
            break
    return release, [s for s in samples if s[0] >= 0.0]


def write_samples(out_dir, samples):
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "samples.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["t_after_release_s", "pitch_deg", "pitch_rate_deg_s", "mean_velocity_m_s"])
        for s in samples:
            writer.writerow([f"{s[0]:.4f}", repr(s[1]), repr(s[2]), repr(s[3])])


def coverage_problem(samples, args):
    """Why the samples cannot be judged (they must start within 0.5 s of the release and reach
    --record), or None."""
    if len(samples) < args.min_samples:
        return f"only {len(samples)} plant samples after the release (need {args.min_samples})"
    if samples[0][0] > 0.5:
        return f"the first sample came {samples[0][0]:.2f} s after the release (need <= 0.5 s)"
    if samples[-1][0] < args.record - 0.05:
        return f"the samples stop {samples[-1][0]:.2f} s after the release (need {args.record} s)"
    return None


def fell_at(samples, tipover_deg):
    """The first time [s after release] |pitch| reached tipover_deg, or None."""
    return next((s[0] for s in samples if abs(s[1]) >= tipover_deg), None)


def scenario_start(args, extra_subscriptions=()):
    return Recorder([("/dynamic_joint_states", DynamicJointState, DEPTH), *extra_subscriptions])


def check_upright(args):
    rec = scenario_start(args)
    release, samples = record_from_release(rec, args)
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    fell = fell_at(samples, args.tipover_deg)
    settled = [s for s in samples if s[0] >= args.settle_from]
    metrics = {
        "samples": len(samples), "release_stamp_s": release,
        "start_pitch_deg": samples[0][1] if samples else None,
        "fell_at_s": fell,
        "max_abs_pitch_deg": max((abs(s[1]) for s in samples), default=math.nan),
        "max_abs_pitch_after_settle_from_deg": max((abs(s[1]) for s in settled), default=math.nan),
        "max_abs_speed_after_settle_from_m_s": max((abs(s[3]) for s in settled), default=math.nan),
        "pitch_at_1s_intervals_deg": [round(s[1], 3) for s in samples
                                      if abs(s[0] - round(s[0])) < 0.005],
        "record_s": args.record, "settle_from_s": args.settle_from,
        "settled_pitch_deg": args.settled_pitch_deg, "max_speed_m_s": args.max_speed_m_s,
        "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1][0] if samples else None,
    }
    print(f"scenario_checks: {len(samples)} samples after release; start pitch "
          f"{metrics['start_pitch_deg']} deg; max |pitch| {metrics['max_abs_pitch_deg']:.3f} deg; "
          f"|pitch| >= {args.tipover_deg} deg at {fell if fell is None else round(fell, 3)} s; "
          f"pitch each second: {metrics['pitch_at_1s_intervals_deg']}")
    if fell is not None:
        # A fall ends the scenario whatever the coverage: there is nothing left to hold.
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged: |pitch| reached {args.tipover_deg} deg {fell:.3f} s "
                      f"after release (the robot fell)")
    problem = coverage_problem(samples, args)
    if problem:
        return finish(args.out, metrics, COULD_NOT_JUDGE, problem)
    worst_pitch = metrics["max_abs_pitch_after_settle_from_deg"]
    worst_speed = metrics["max_abs_speed_after_settle_from_m_s"]
    if worst_pitch > args.settled_pitch_deg:
        return finish(args.out, metrics, FAIL,
                      f"|pitch| reached {worst_pitch:.3f} deg between {args.settle_from} and "
                      f"{args.record} s (limit {args.settled_pitch_deg} deg)")
    if worst_speed > args.max_speed_m_s:
        return finish(args.out, metrics, FAIL,
                      f"the measured wheel speed reached {worst_speed:.4f} m/s between "
                      f"{args.settle_from} and {args.record} s (limit {args.max_speed_m_s} m/s)")
    return finish(args.out, metrics, PASS,
                  f"held for {args.record} s: |pitch| <= {worst_pitch:.3f} deg and |v| <= "
                  f"{worst_speed:.4f} m/s from {args.settle_from} s on")


def check_impulse(args):
    rec = scenario_start(args)
    release, samples = record_from_release(rec, args)
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    # The push: the largest one-sample jump in the exported pitch_rate (undelayed, sim-plant.md
    # §3.7) near the push time. Exported rate jumps by +impulse_deg_s whatever pitch_sign is (§5).
    near = [(b[0], b[2] - a[2]) for a, b in zip(samples, samples[1:])
            if args.impulse_time_s - 0.05 <= b[0] <= args.impulse_time_s + 0.1]
    jump_t, jump = max(near, key=lambda x: x[1], default=(None, math.nan))
    fell = fell_at(samples, args.tipover_deg)
    after = [s for s in samples if s[0] >= args.impulse_time_s]
    # The last time after the push that |pitch| was above the settled limit.
    last_out = max((s[0] for s in after if abs(s[1]) > args.settled_pitch_deg), default=None)
    settle_s = (last_out - args.impulse_time_s) if last_out is not None else 0.0
    metrics = {
        "samples": len(samples), "release_stamp_s": release,
        "impulse_seen_at_s": jump_t, "impulse_rate_jump_deg_s": jump,
        "fell_at_s": fell,
        "peak_abs_pitch_after_impulse_deg": max((abs(s[1]) for s in after), default=math.nan),
        "max_abs_pitch_before_impulse_deg": max((abs(s[1]) for s in samples
                                                 if s[0] < args.impulse_time_s), default=math.nan),
        "settle_time_after_impulse_s": settle_s,
        "record_s": args.record, "impulse_time_s": args.impulse_time_s,
        "impulse_deg_s": args.impulse_deg_s, "settle_within_s": args.settle_within_s,
        "settled_pitch_deg": args.settled_pitch_deg, "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1][0] if samples else None,
    }
    print(f"scenario_checks: {len(samples)} samples after release; rate jump {jump:.1f} deg/s at "
          f"{jump_t} s (push {args.impulse_deg_s} deg/s at {args.impulse_time_s} s); peak |pitch| "
          f"after it {metrics['peak_abs_pitch_after_impulse_deg']:.3f} deg; last |pitch| > "
          f"{args.settled_pitch_deg} deg {settle_s:.3f} s after the push; |pitch| >= "
          f"{args.tipover_deg} deg at {fell if fell is None else round(fell, 3)} s")
    if fell is not None and fell < args.impulse_time_s:
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged before the push: |pitch| reached {args.tipover_deg} deg "
                      f"{fell:.3f} s after release")
    if not jump >= 0.8 * args.impulse_deg_s:
        if fell is None and coverage_problem(samples, args):
            return finish(args.out, metrics, COULD_NOT_JUDGE, coverage_problem(samples, args))
        return finish(args.out, metrics, FAIL,
                      f"no push seen: the largest pitch_rate jump near {args.impulse_time_s} s was "
                      f"{jump:.1f} deg/s (need >= {0.8 * args.impulse_deg_s:g})")
    if fell is not None:
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged after the push: |pitch| reached {args.tipover_deg} deg "
                      f"{fell - args.impulse_time_s:.3f} s after it (the robot fell)")
    problem = coverage_problem(samples, args)
    if problem:
        return finish(args.out, metrics, COULD_NOT_JUDGE, problem)
    if settle_s > args.settle_within_s:
        return finish(args.out, metrics, FAIL,
                      f"|pitch| was still above {args.settled_pitch_deg} deg {settle_s:.3f} s after "
                      f"the push (limit {args.settle_within_s} s)")
    return finish(args.out, metrics, PASS,
                  f"recovered: |pitch| back under {args.settled_pitch_deg} deg {settle_s:.3f} s after "
                  f"a {args.impulse_deg_s} deg/s push (peak "
                  f"{metrics['peak_abs_pitch_after_impulse_deg']:.3f} deg) and stayed there")


def check_velocity_step(args):
    flag_topic = "/diff_drive_controller/command_timed_out"
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rec = scenario_start(args, [(flag_topic, Bool, latched)])
    gamepad = rec.create_publisher(TwistStamped, "/diffbot_base_controller/cmd_vel", 10)
    command = TwistStamped()
    state = {"next": time.monotonic(), "step_wall_s": None, "published": 0}

    def publish(t_after_release):
        """Keep the gamepad going at --rate-hz (DiffDriveController's watchdog needs it): 0 until
        --before-s after release, then the step. Before release (None) it sends 0."""
        now = time.monotonic()
        if now < state["next"]:
            return
        state["next"] = now + 1.0 / args.rate_hz
        stepping = t_after_release is not None and t_after_release >= args.before_s
        if stepping and state["step_wall_s"] is None:
            state["step_wall_s"] = time.time()  # system clock: the same clock as the stamps
        command.twist.linear.x = args.speed_m_s if stepping else 0.0
        command.header.stamp = rec.get_clock().now().to_msg()
        gamepad.publish(command)
        state["published"] += 1

    args.record = args.before_s + args.step_s
    release, samples = record_from_release(rec, args, during=publish)
    flags = [m.data for _, m in rec.received[flag_topic]]
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    if state["step_wall_s"] is None:
        return finish(args.out, {"samples": len(samples)}, COULD_NOT_JUDGE, "the step never started")
    step_t = state["step_wall_s"] - release  # [s after release] the first step command left
    after = [s for s in samples if s[0] >= step_t]
    target = args.rise_fraction * args.speed_m_s
    rise_at = next((s[0] for s in after if s[3] >= target), None)
    rise_s = rise_at - step_t if rise_at is not None else None
    peak_v = max((s[3] for s in after), default=math.nan)
    overshoot_pct = 100.0 * (peak_v - args.speed_m_s) / args.speed_m_s
    peak_pitch = max((abs(s[1]) for s in samples), default=math.nan)
    fell = fell_at(samples, args.tipover_deg)
    metrics = {
        "samples": len(samples), "release_stamp_s": release, "gamepad_messages": state["published"],
        "timed_out_flags_seen": flags, "step_at_s": step_t, "rise_time_s": rise_s,
        "peak_mean_velocity_m_s": peak_v, "overshoot_pct": overshoot_pct,
        "min_mean_velocity_after_step_m_s": min((s[3] for s in after), default=math.nan),
        "final_mean_velocity_m_s": samples[-1][3] if samples else None,
        "peak_abs_pitch_deg": peak_pitch, "fell_at_s": fell,
        "speed_m_s": args.speed_m_s, "rise_fraction": args.rise_fraction,
        "rise_within_s": args.rise_within_s, "max_overshoot_pct": args.max_overshoot_pct,
        "max_pitch_deg": args.max_pitch_deg, "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1][0] if samples else None,
    }
    print(f"scenario_checks: step to {args.speed_m_s} m/s at {step_t:.3f} s after release; "
          f"{args.rise_fraction:.0%} reached after {rise_s if rise_s is None else round(rise_s, 3)} s; "
          f"peak mean v {peak_v:.4f} m/s (overshoot {overshoot_pct:.1f} %), lowest "
          f"{metrics['min_mean_velocity_after_step_m_s']:.4f} m/s; peak |pitch| {peak_pitch:.3f} deg; "
          f"|pitch| >= {args.tipover_deg} deg at {fell if fell is None else round(fell, 3)} s")
    if False in flags and True in flags[flags.index(False) + 1:]:
        print("scenario_checks: note: command_timed_out went True while the gamepad was driving "
              "(a delivery gap longer than the watchdog's timeout)")
    if fell is not None:
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged: |pitch| reached {args.tipover_deg} deg {fell:.3f} s after "
                      f"release ({fell - step_t:.3f} s after the step; the robot fell)")
    problem = coverage_problem(samples, args)
    if problem:
        return finish(args.out, metrics, COULD_NOT_JUDGE, problem)
    if rise_s is None or rise_s > args.rise_within_s:
        return finish(args.out, metrics, FAIL,
                      f"the mean measured velocity did not reach {target:g} m/s within "
                      f"{args.rise_within_s} s of the step (reached after {rise_s} s)")
    if overshoot_pct > args.max_overshoot_pct:
        return finish(args.out, metrics, FAIL,
                      f"overshoot {overshoot_pct:.1f} % (limit {args.max_overshoot_pct} %)")
    if peak_pitch > args.max_pitch_deg:
        return finish(args.out, metrics, FAIL,
                      f"|pitch| reached {peak_pitch:.3f} deg (limit {args.max_pitch_deg} deg)")
    return finish(args.out, metrics, PASS,
                  f"followed the step: {args.rise_fraction:.0%} in {rise_s:.3f} s, overshoot "
                  f"{overshoot_pct:.1f} %, peak |pitch| {peak_pitch:.3f} deg")


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

    # The balance scenarios. Every threshold is required: verify_sim.sh holds the numbers and why.
    def scenario(name, function, record_help):
        p = add(name, function, record_help)
        p.add_argument("--launch-log", required=True,
                       help="rocko_sim.launch.py's log, for SimPlant's release stamp (t = 0)")
        p.add_argument("--tipover-deg", type=float, required=True,
                       help="|pitch| at or past this [deg] counts as a fall")
        return p

    p = scenario("upright", check_upright, "seconds to record after release")
    p.add_argument("--settle-from", type=float, required=True,
                   help="judge pitch and speed from this many seconds after release [s]")
    p.add_argument("--settled-pitch-deg", type=float, required=True, help="limit on |pitch| [deg]")
    p.add_argument("--max-speed-m-s", type=float, required=True,
                   help="limit on the mean measured wheel speed [m/s]")
    p = scenario("impulse", check_impulse, "seconds to record after release")
    p.add_argument("--impulse-time-s", type=float, required=True,
                   help="the launch's impulse_time_s [s after release]")
    p.add_argument("--impulse-deg-s", type=float, required=True,
                   help="the launch's impulse_angular_velocity_deg_per_s [deg/s]")
    p.add_argument("--settle-within-s", type=float, required=True,
                   help="|pitch| must be back under --settled-pitch-deg this soon after the push [s]")
    p.add_argument("--settled-pitch-deg", type=float, required=True, help="limit on |pitch| [deg]")
    p = scenario("velocity-step", check_velocity_step, None)  # its timing is --before-s, --step-s
    p.add_argument("--speed-m-s", type=float, required=True, help="the step's linear.x [m/s]")
    p.add_argument("--rate-hz", type=float, default=20.0, help="fake gamepad rate [Hz]")
    p.add_argument("--before-s", type=float, required=True,
                   help="linear.x = 0 this long after release, then the step [s]")
    p.add_argument("--step-s", type=float, required=True, help="how long the step is held [s]")
    p.add_argument("--rise-fraction", type=float, required=True, help="e.g. 0.9 for 90 %%")
    p.add_argument("--rise-within-s", type=float, required=True, help="[s after the step]")
    p.add_argument("--max-overshoot-pct", type=float, required=True, help="[%% of the step]")
    p.add_argument("--max-pitch-deg", type=float, required=True, help="limit on |pitch| [deg]")
    args = parser.parse_args()

    rclpy.init()
    try:
        return args.function(args)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
