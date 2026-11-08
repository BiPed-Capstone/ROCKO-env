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
                /dynamic_joint_states keeps arriving (the plant is still being simulated). All
                times are SEND times (DDS source timestamps), and the watchdog is judged on every
                gap between commands, because this script itself can stall and send one late. A
                command counts only once it has certainly left (publish() returned) and the node
                has run two cycles since, because both a publish() call and the node itself have
                been seen to stall ~0.2-0.3 s in the container.
    feedforward-sends  for check FF (finding H-11): records when each /left_feedforward and
                /right_feedforward message was sent, so verify_sim.sh can time each controller's
                time-out from the last one. Exit 0 if both wheels were heard.
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
                   under --max-pitch-deg, and never reaches --tipover-deg. The overshoot is the
                   peak of the measured velocity recomputed with each control cycle's real length
                   (velocity_from_true_periods), because a late cycle in the container makes the
                   encoder replica read ~20 % high for a moment; the exported peak is recorded too.
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
from collections import namedtuple

import rclpy
from control_msgs.msg import DynamicJointState, MultiDOFCommand, MultiDOFStateStamped
from geometry_msgs.msg import TwistStamped
from pal_statistics_msgs.msg import Statistics
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64

JOINTS = ("left_wheel_joint", "right_wheel_joint")
PASS, FAIL, COULD_NOT_JUDGE = 0, 1, 2
# Queue depth 200 = 2 s of a 100 Hz stream, so a slow moment in this script drops nothing.
DEPTH = 200


class Recorder(Node):
    """Subscribes to the topics it is given and keeps every message with the time it ARRIVED
    (time.monotonic(), seconds; the same clock this script uses for anything it publishes).

    It also keeps, in sent_s[topic], the time each message was SENT: the DDS source timestamp the
    publisher's middleware puts on it, in seconds on the system clock (time.time()). Arrival times
    are taken by this script, so if this process stalls (a Python garbage collection, a slow
    moment in the container) every message that came in meanwhile gets the same late arrival time.
    The sent time does not move. (Only check_teleop reads sent_s. The checks that pop messages
    off received[topic] as they go leave sent_s alone, so for them the two lists do not line up.)"""

    def __init__(self, subscriptions):
        super().__init__("verify_sim_scenario_checks")
        self.received = {}
        self.sent_s = {}
        for topic, msg_type, qos in subscriptions:
            self.received[topic] = []
            self.sent_s[topic] = []
            # Two parameters, so rclpy passes the message info (with source_timestamp) as well.
            self.create_subscription(
                msg_type, topic, lambda msg, info, t=topic: self._keep(t, msg, info), qos)

    def _keep(self, topic, msg, info):
        self.received[topic].append((time.monotonic(), msg))
        self.sent_s[topic].append(info["source_timestamp"] * 1e-9)  # [ns] -> [s]

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
# A command counts as processed by DiffDriveController once the node has run this many of its 10 ms
# timer cycles after the command left. Why two: see handled_by in check_teleop.
NODE_CYCLES_TO_HANDLE = 2


def check_teleop(args):
    flag_topic = "/diff_drive_controller/command_timed_out"
    reference_topic = "/velocity_pid_controller/reference"
    plant_topic = "/dynamic_joint_states"
    gamepad_topic = "/diffbot_base_controller/cmd_vel"
    # transient_local, as the publisher: the current state arrives even though we start late.
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # This script also listens to its own gamepad topic, for the source timestamp of each command:
    # the moment the middleware actually sent it (see Recorder).
    rec = Recorder([(flag_topic, Bool, latched), (reference_topic, MultiDOFCommand, DEPTH),
                    (plant_topic, DynamicJointState, DEPTH), (gamepad_topic, TwistStamped, DEPTH)])
    gamepad = rec.create_publisher(TwistStamped, gamepad_topic, 10)

    # Wait for DiffDriveController: its flag has arrived and it subscribes to the gamepad topic
    # (2 subscribers: it, and this script's own).
    ready = rec.spin_until(
        lambda: rec.received[flag_topic] and gamepad.get_subscription_count() >= 2, args.wait)
    if not ready:
        rec.destroy_node()
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"DiffDriveController did not appear within {args.wait} s")
    rec.spin_for(0.5)  # half a second with no gamepad: the flag should say timed out

    # Play the gamepad: the same command every 1/rate s, for drive_s seconds.
    command = TwistStamped()
    command.twist.linear.x = args.speed_m_s
    commands = round(args.drive_s * args.rate_hz)
    next_publish = time.monotonic()
    command_left_s = []  # [system clock s] when each publish() call returned: the command had left
    for _ in range(commands):
        rec.spin_until(lambda: time.monotonic() >= next_publish, 1.0)
        command.header.stamp = rec.get_clock().now().to_msg()
        gamepad.publish(command)
        command_left_s.append(time.time())
        next_publish += 1.0 / args.rate_hz
    rec.spin_for(args.after_s)  # ... and then the gamepad goes silent
    end_s = time.time()
    rec.destroy_node()
    # [system clock s] when each command was stamped as sent: its DDS source timestamp, heard back.
    command_sent_s = sorted(rec.sent_s[gamepad_topic])
    if len(command_sent_s) != commands:
        return finish(args.out, {"commands_published": commands, "commands_heard": len(command_sent_s)},
                      COULD_NOT_JUDGE, f"sent {commands} gamepad commands but heard "
                      f"{len(command_sent_s)} of them back, so their send times are not all known")

    # Every time from here on is on the system clock, in seconds, and is a time a message was SENT
    # (its source timestamp), or the time one of our publish() calls returned. Not arrival times:
    # this process can stall (0.35 s seen in the container, 2026-11-03), and everything that
    # arrives during a stall would get the same late time.
    def sent(topic):
        return list(zip(rec.sent_s[topic], (m for _, m in rec.received[topic])))
    flags, references, plant = sent(flag_topic), sent(reference_topic), sent(plant_topic)
    first_pub, last_pub = command_sent_s[0], command_sent_s[-1]
    settled = last_pub + args.timeout_s + args.margin_s  # by now the reference must be zero
    early_s = args.timeout_s - 0.01  # less one 10 ms timer period of rounding
    late_s = args.timeout_s + args.margin_s
    # DiffDriveController publishes the velocity reference once in every 10 ms timer cycle, so
    # the send times of the references are the moments the node was demonstrably running.
    node_cycles = sorted(t for t, _ in references)

    # What this script can know about one command: it reached the node no sooner than its send
    # stamp, and it had left once publish() returned. Usually that is under 1 ms later, but the
    # middleware stamps a message and then sends it, and in the container a publish() has been
    # seen to block 0.30 s in between (2026-11-07). And the node can only take a command off its
    # queue while it runs: a DiffDriveController that is itself stalled processes nothing, and on
    # waking its timer may run before the waiting command (seen: True, then False 1.3 ms later).
    # So a command counts as HANDLED by time t only if the node ran NODE_CYCLES_TO_HANDLE (two)
    # timer cycles after publish() returned and before t. Why two, step by step:
    #   1. the first cycle after the command left may run before the command is in the node's
    #      queue, or together with it: when the timer and a waiting command are ready at once,
    #      rclpy runs the timer first, so that cycle can still check the watchdog without it;
    #   2. before the executor waits again it runs every callback that is ready, the command's too;
    #   3. so by the second timer cycle the command's callback has run, and that cycle sees it.
    def handled_by(t):
        """Send stamps of the commands the node had certainly processed before time t."""
        return [sent_s for sent_s, left_s in zip(command_sent_s, command_left_s)
                if sum(1 for c in node_cycles if left_s <= c < t) >= NODE_CYCLES_TO_HANDLE]

    # The rule, judged on every silence (the gaps between commands and the silence after the
    # last one), because a stall can hold back a command and make a gap mid-drive:
    #   - a True may never come sooner than early_s after the latest command HANDLED before it
    #     (a correct watchdog fires >= timeout after its latest command arrived, which is no
    #     sooner than that command's send stamp);
    #   - in every certain silence longer than late_s (from the moment a command had certainly
    #     left to the earliest the next one could arrive) the node must be timed out by its first
    #     cycle late_s after the command had left. The final silence is always one (after_s 2 s).
    gaps = [b - a for a, b in zip(command_sent_s, command_sent_s[1:])]
    early = []  # [s after the latest handled command] for each True sent too soon
    for t, m in flags:
        if m.data and t > first_pub:
            handled = handled_by(t)
            if handled and t - handled[-1] < early_s:
                early.append(round(t - handled[-1], 4))
    # (sent_s, left_s) of each command followed by a certain silence longer than late_s.
    silences = [(sent_s, left_s) for sent_s, left_s, next_sent_s
                in zip(command_sent_s, command_left_s, command_sent_s[1:] + [math.inf])
                if next_sent_s - left_s > late_s]
    missed = []  # [s into the drive] for each silence the node did not time out in
    for sent_s, left_s in silences:
        deadline = next((c for c in node_cycles if c >= left_s + late_s), None)
        if deadline is None or not any(m.data and sent_s < t <= deadline for t, m in flags):
            missed.append(round(sent_s - first_pub, 3))
    mid_drive_silences = [s for s in silences if s[0] != last_pub]  # all but the final one, for the note

    went_false = [t for t, m in flags if not m.data and first_pub <= t <= last_pub]
    went_true = [t for t, m in flags if m.data and t > last_pub]
    latency = went_true[0] - last_pub if went_true else None  # reported; judged by the two rules
    driven_values = [list(m.values) for t, m in references if first_pub < t <= last_pub]
    after_values = [list(m.values) for t, m in references if t > settled]
    plant_after = [t for t, _ in plant if t > settled]
    metrics = {
        "commands_published": len(command_sent_s),
        "drive_s": last_pub - first_pub,
        "largest_command_gap_s": max(gaps, default=None),
        "command_gaps_longer_than_timeout_plus_margin": len(mid_drive_silences),
        "flag_messages": [(round(t - first_pub, 4), m.data) for t, m in flags],
        "timeout_latency_s": latency,
        "timeout_s": args.timeout_s,
        "margin_s": args.margin_s,
        "timed_out_too_early_s": early,
        "silences_without_a_timeout_s": missed,
        # How long publish() took after the middleware stamped a command, and the longest the node
        # published nothing while it should have been running every 10 ms: the two delays the rules
        # allow for. Both are usually a few ms.
        "longest_publish_call_s": max(left_s - sent_s
                                      for sent_s, left_s in zip(command_sent_s, command_left_s)),
        "longest_node_cycle_gap_s": max(
            (b - a for a, b in zip(node_cycles, node_cycles[1:]) if first_pub - 0.5 < a < end_s),
            default=None),
        "references_while_driven": len(driven_values),
        "references_while_driven_equal_to_command": sum(
            v == [args.speed_m_s] * 2 for v in driven_values),
        "references_after_timeout": len(after_values),
        "nonzero_references_after_timeout": sum(v != [0.0, 0.0] for v in after_values),
        "plant_samples_after_timeout": len(plant_after),
        "after_window_s": end_s - settled,
    }
    print(f"scenario_checks: {len(command_sent_s)} gamepad commands over {metrics['drive_s']:.2f} s "
          f"(largest gap {max(gaps, default=0.0):.3f} s; longest publish() call "
          f"{metrics['longest_publish_call_s']:.4f} s; longest gap between the node's cycles "
          f"{metrics['longest_node_cycle_gap_s']:.3f} s); "
          f"flag (s after first command, value): {metrics['flag_messages']}")
    if mid_drive_silences:
        print(f"scenario_checks: note: {len(mid_drive_silences)} silence(s) longer than {late_s:g} s in the "
              f"middle of the drive (this script stalled, or a publish() call blocked: not the node "
              f"under test); the watchdog must time out in each, and it is judged so")
    print(f"scenario_checks: timed out {latency if latency is None else round(latency, 4)} s after "
          f"the last command (need {args.timeout_s:g} to {late_s:g} s); "
          f"references while driven {len(driven_values)} "
          f"({metrics['references_while_driven_equal_to_command']} equal to the command), after "
          f"the timeout {len(after_values)} ({metrics['nonzero_references_after_timeout']} "
          f"non-zero); plant samples after the timeout {len(plant_after)}")

    if not went_false:
        return finish(args.out, metrics, FAIL, "command_timed_out never went False while driven")
    if early:
        return finish(args.out, metrics, FAIL,
                      f"command_timed_out went True only {early} s after a command the node had "
                      f"processed, sooner than the {args.timeout_s:g} s timeout")
    if latency is None:
        return finish(args.out, metrics, FAIL,
                      f"command_timed_out never went True in the {args.after_s} s after the last "
                      "command")
    if missed:
        return finish(args.out, metrics, FAIL,
                      f"the gamepad went silent for longer than {late_s:g} s at {missed} s into the "
                      f"drive and command_timed_out was not True within {late_s:g} s of it (first "
                      f"True after the last command: {latency:.3f} s)")
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


# ---- H-11: when the feedforward messages were really sent ---------------------------------------
def check_feedforward_sends(args):
    """Not a judgement of its own: records the source timestamp of every message on
    /left_feedforward and /right_feedforward for --record s, for verify_sim.sh's check FF. FF times
    each controller's time-out from the LAST MESSAGE REALLY SENT on that wheel. Timing it from the
    moment the publisher was killed is wrong whenever the publisher stalled just before the kill: its
    last message then left earlier than the kill, and a correct 200 ms limit looks too short."""
    topics = {side: f"/{side}_feedforward" for side in ("left", "right")}
    rec = Recorder([(topic, Float64, DEPTH) for topic in topics.values()])
    rec.spin_for(args.record)
    rec.destroy_node()
    metrics = {"gap_s": args.gap_s}
    for side, topic in topics.items():
        sent = sorted(rec.sent_s[topic])
        # Sender-side gaps: the publisher itself sent nothing for longer than --gap-s.
        gaps = [(round(a, 3), round(b - a, 3)) for a, b in zip(sent, sent[1:]) if b - a > args.gap_s]
        metrics[f"{side}_messages"] = len(sent)
        metrics[f"{side}_last_sent_s"] = sent[-1] if sent else None
        metrics[f"{side}_send_gaps"] = gaps  # (start [s, system clock], length [s])
        print(f"scenario_checks: {side}: {len(sent)} feedforward messages heard, "
              f"{len(gaps)} send gap(s) longer than {args.gap_s:g} s {gaps}")
    if min(metrics["left_messages"], metrics["right_messages"]) < args.min_samples:
        return finish(args.out, metrics, COULD_NOT_JUDGE,
                      f"fewer than {args.min_samples} feedforward messages heard on a wheel")
    return finish(args.out, metrics, PASS, "recorded the send time of every feedforward message")


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

# Rules of thumb for judging the samples (they are not pass thresholds; those are verify_sim.sh's):
# a push counts as "seen" if the rate jump is at least this fraction of the commanded push, since
# the 100 Hz samples may straddle it.
PUSH_SEEN_FRACTION = 0.8
# Where to look for the push's rate jump, around the launch's impulse_time_s [s after release]:
# a little before (the clocks differ slightly) and longer after (the samples arrive late).
PUSH_WINDOW_S = (-0.05, 0.1)
# The plant stream must start this soon after the release, or the start of the run was missed.
MAX_FIRST_SAMPLE_S = 0.5
# The last sample may stop this far short of the record length (one 100 Hz period is 0.01 s).
END_SLACK_S = 0.05
# The encoder replica (finding U-2, SimPlantModel.hpp update_encoder_outputs, copied from
# QuadEncoders.py): each cycle it divides the wheel travel by an ASSUMED period and low-passes it.
# Copied from SimPlantModel.hpp (kEncoderAssumedPeriodS, kEncoderFilterAlpha); keep them equal.
REPLICA_ASSUMED_PERIOD_S = 0.01
REPLICA_FILTER_ALPHA = 0.05
# One plant sample of the balance scenarios, from one /dynamic_joint_states message. pitch, its rate
# and the wheel travel are left_wheel_joint's (one IMU and one encoder reply feed both joints).
#   t_s                 [s after SimPlant's release] the message's header stamp minus the release stamp
#   pitch_deg           exported pitch [deg], as the controllers read it
#   pitch_rate_deg_s    exported pitch_rate [deg/s] (the gyro, not delayed)
#   mean_velocity_m_s   the mean of the two wheels' exported `velocity` [m/s], what the velocity loop
#                       reads (see the docstring above: about half the true speed, finding U-2)
#   position_m          the exported `position` [m]: the wheel travel velocity_from_true_periods() needs
#   right_velocity_m_s  the right wheel's exported `velocity` [m/s] (the replica's filter state)
Sample = namedtuple("Sample", "t_s pitch_deg pitch_rate_deg_s mean_velocity_m_s position_m right_velocity_m_s")
# The column names of samples.csv, in Sample's order (the first says what t_s counts from).
SAMPLES_CSV_HEADER = ["t_after_release_s", "pitch_deg", "pitch_rate_deg_s", "mean_velocity_m_s",
                      "position_m", "right_velocity_m_s"]

# velocity_from_true_periods() replays one replica cycle from the previous sample; if that gives this
# sample's exported velocity to within this [m/s], exactly one cycle lay between them. Doubles
# arrive unchanged, so only float noise is allowed.
REPLAY_TOLERANCE_M_S = 1e-6
# If more than this fraction of sample pairs are not one replica cycle apart, the replica is not the
# one written in velocity_from_true_periods() and V cannot judge. Dropped samples (this script's
# subscription loses messages when it stalls) were 1 pair in ~880 in the runs of 2026-11-07; a
# changed replica would miss on nearly every pair.
MAX_NOT_ONE_CYCLE_FRACTION = 0.05


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


def record_from_release(rec, args, record_s, topic="/dynamic_joint_states", during=None):
    """Spin until record_s s of plant samples after the release have arrived. Returns (release
    stamp or None, [Sample, ...] from the release on). during(t_s) is called while spinning with
    the latest sample's time, or None before the first (velocity-step publishes from it)."""
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
            samples.append(Sample(
                t_s=stamp_s(msg) - release, pitch_deg=left.get("pitch", math.nan),
                pitch_rate_deg_s=left.get("pitch_rate", math.nan), mean_velocity_m_s=v,
                position_m=left.get("position", math.nan),
                right_velocity_m_s=right.get("velocity", math.nan)))

    end = time.monotonic() + record_s + args.wait
    while time.monotonic() < end:
        rec.spin_for(0.02)
        collect_new()
        if during:
            during(samples[-1].t_s if samples else None)
        if samples and samples[-1].t_s >= record_s:
            break
    return release, [s for s in samples if s.t_s >= 0.0]


def write_samples(out_dir, samples):
    """samples.csv: one row per Sample, the time to 0.1 ms, every other value exactly (repr)."""
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "samples.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(SAMPLES_CSV_HEADER)
        for s in samples:
            writer.writerow([f"{s.t_s:.4f}"] + [repr(x) for x in s[1:]])


def coverage_problem(samples, args, record_s):
    """Why the samples cannot be judged (they must start within MAX_FIRST_SAMPLE_S of the release
    and reach record_s), or None."""
    if len(samples) < args.min_samples:
        return f"only {len(samples)} plant samples after the release (need {args.min_samples})"
    if samples[0].t_s > MAX_FIRST_SAMPLE_S:
        return (f"the first sample came {samples[0].t_s:.2f} s after the release "
                f"(need <= {MAX_FIRST_SAMPLE_S} s)")
    if samples[-1].t_s < record_s - END_SLACK_S:
        return f"the samples stop {samples[-1].t_s:.2f} s after the release (need {record_s} s)"
    return None


def fell_at(samples, tipover_deg):
    """The first time [s after release] |pitch| reached tipover_deg, or None."""
    return next((s.t_s for s in samples if abs(s.pitch_deg) >= tipover_deg), None)


def nearest(samples, t_s):
    """The sample whose time is closest to t_s."""
    return min(samples, key=lambda s: abs(s.t_s - t_s))


def velocity_from_true_periods(samples):
    """The mean measured wheel velocity [m/s] the encoder replica WOULD export if it divided each
    cycle's travel by that cycle's real length instead of an assumed 10 ms; one value per sample.
    Returns (values, problem, gaps): problem is None, or why the values cannot be trusted; gaps is
    the number of sample pairs with dropped samples between them.

    Why V needs it: when controller_manager runs a cycle late in the container (26-30 ms instead
    of 10 ms, seen in the kept V runs), the replica divides ~3 cycles of travel by 0.01 s, so its
    output jumps ~20 % in one sample and then decays over its own low-pass (time constant
    0.01 / 0.05 = 0.2 s). That jump is the replica's timing error, not the wheel going faster.
    The real cycle length is the gap between the two samples' stamps (the controller_manager's
    clock, the same period SimPlant integrated the physics over).

    The replica, per cycle (sim-plant.md §3.7), with a = REPLICA_FILTER_ALPHA:
        left  = (1 - a) * filter + a * travel / 0.01     (exported as the left wheel's velocity)
        right = (1 - a) * left                           (the second call sees no travel)
    and the exported mean is (left + right) / 2. Dividing by the cycle's real length instead
    changes `left` by a * travel * (1/0.01 - 1/period), and that difference then decays through
    the same filter. So this function carries `error` = how much the replica's filter reads above
    the true-period filter, and subtracts it from the exported value.

    If samples were dropped between two samples (this script's subscription loses messages when
    it stalls), the cycles in between are unknown: the error is decayed as over that many nominal
    cycles and nothing else is removed, so any artefact inside the gap stays in (it can only make
    the overshoot look larger, never hide one)."""
    a = REPLICA_FILTER_ALPHA
    if any(b.t_s <= s.t_s for s, b in zip(samples, samples[1:])):
        return [], "two plant samples have the same or decreasing stamps, so a period is unknown", 0
    corrected = [samples[0].mean_velocity_m_s]
    error = 0.0  # [m/s] the replica's filter (its right-wheel output) minus the true-period one
    not_one_cycle = 0
    for before, s in zip(samples, samples[1:]):
        travel_m, period_s = s.position_m - before.position_m, s.t_s - before.t_s
        # One replica cycle from the previous sample's state (its right velocity is the filter):
        left = (1 - a) * before.right_velocity_m_s + a * travel_m / REPLICA_ASSUMED_PERIOD_S
        if abs(0.5 * (left + (1 - a) * left) - s.mean_velocity_m_s) <= REPLAY_TOLERANCE_M_S:
            error_left = (1 - a) * error + a * travel_m * (1 / REPLICA_ASSUMED_PERIOD_S - 1 / period_s)
        else:  # samples dropped in between: about period_s / 0.01 cycles, at least two
            not_one_cycle += 1
            cycles = max(2, round(period_s / REPLICA_ASSUMED_PERIOD_S))
            # Each cycle applies the filter twice (the left call, then the right), so `cycles`
            # cycles decay the error by (1 - a) ** (2 * cycles). The last factor is the right call
            # of the final cycle, applied by the line after this if/else, as for one cycle.
            error_left = error * (1 - a) ** (2 * cycles - 1)
        error = (1 - a) * error_left
        corrected.append(s.mean_velocity_m_s - 0.5 * (error_left + error))
    if not_one_cycle > MAX_NOT_ONE_CYCLE_FRACTION * (len(samples) - 1):
        return corrected, (f"{not_one_cycle} of {len(samples) - 1} sample pairs are not one encoder-"
                           f"replica cycle apart (allowed {MAX_NOT_ONE_CYCLE_FRACTION:.0%}): the "
                           "replica is not the one in velocity_from_true_periods()"), not_one_cycle
    return corrected, None, not_one_cycle


def check_upright(args):
    rec = Recorder([("/dynamic_joint_states", DynamicJointState, DEPTH)])
    release, samples = record_from_release(rec, args, args.record)
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    fell = fell_at(samples, args.tipover_deg)
    settled = [s for s in samples if s.t_s >= args.settle_from]
    metrics = {
        "samples": len(samples), "release_stamp_s": release,
        "start_pitch_deg": samples[0].pitch_deg if samples else None,
        "fell_at_s": fell,
        "max_abs_pitch_deg": max((abs(s.pitch_deg) for s in samples), default=math.nan),
        "max_abs_pitch_after_settle_from_deg": max((abs(s.pitch_deg) for s in settled),
                                                   default=math.nan),
        "max_abs_speed_after_settle_from_m_s": max((abs(s.mean_velocity_m_s) for s in settled),
                                                   default=math.nan),
        # The sample nearest each whole second (1, 2, ...), for a quick look at the trend.
        "pitch_at_1s_intervals_deg": [round(nearest(samples, k).pitch_deg, 3)
                                      for k in range(1, int(args.record) + 1)] if samples else [],
        "record_s": args.record, "settle_from_s": args.settle_from,
        "settled_pitch_deg": args.settled_pitch_deg, "max_speed_m_s": args.max_speed_m_s,
        "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1].t_s if samples else None,
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
    problem = coverage_problem(samples, args, args.record)
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
    rec = Recorder([("/dynamic_joint_states", DynamicJointState, DEPTH)])
    release, samples = record_from_release(rec, args, args.record)
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    # The push: the largest one-sample jump in the exported pitch_rate (undelayed, sim-plant.md
    # §3.7) near the push time. Exported rate jumps by +impulse_deg_s whatever pitch_sign is (§5).
    near = [(b.t_s, b.pitch_rate_deg_s - a.pitch_rate_deg_s) for a, b in zip(samples, samples[1:])
            if args.impulse_time_s + PUSH_WINDOW_S[0] <= b.t_s <= args.impulse_time_s + PUSH_WINDOW_S[1]]
    jump_t, jump = max(near, key=lambda x: x[1], default=(None, math.nan))
    fell = fell_at(samples, args.tipover_deg)
    after = [s for s in samples if s.t_s >= args.impulse_time_s]
    # The last time after the push that |pitch| was above the settled limit.
    last_out = max((s.t_s for s in after if abs(s.pitch_deg) > args.settled_pitch_deg), default=None)
    settle_s = (last_out - args.impulse_time_s) if last_out is not None else 0.0
    metrics = {
        "samples": len(samples), "release_stamp_s": release,
        "impulse_seen_at_s": jump_t, "impulse_rate_jump_deg_s": jump,
        "fell_at_s": fell,
        "peak_abs_pitch_after_impulse_deg": max((abs(s.pitch_deg) for s in after), default=math.nan),
        "max_abs_pitch_before_impulse_deg": max((abs(s.pitch_deg) for s in samples
                                                 if s.t_s < args.impulse_time_s), default=math.nan),
        "settle_time_after_impulse_s": settle_s,
        "record_s": args.record, "impulse_time_s": args.impulse_time_s,
        "impulse_deg_s": args.impulse_deg_s, "settle_within_s": args.settle_within_s,
        "settled_pitch_deg": args.settled_pitch_deg, "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1].t_s if samples else None,
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
    if not jump >= PUSH_SEEN_FRACTION * args.impulse_deg_s:
        if fell is None and coverage_problem(samples, args, args.record):
            return finish(args.out, metrics, COULD_NOT_JUDGE,
                          coverage_problem(samples, args, args.record))
        return finish(args.out, metrics, FAIL,
                      f"no push seen: the largest pitch_rate jump near {args.impulse_time_s} s was "
                      f"{jump:.1f} deg/s (need >= {PUSH_SEEN_FRACTION * args.impulse_deg_s:g})")
    if fell is not None:
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged after the push: |pitch| reached {args.tipover_deg} deg "
                      f"{fell - args.impulse_time_s:.3f} s after it (the robot fell)")
    problem = coverage_problem(samples, args, args.record)
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
    rec = Recorder([("/dynamic_joint_states", DynamicJointState, DEPTH), (flag_topic, Bool, latched)])
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

    record_s = args.before_s + args.step_s  # this scenario's length is the sum, not --record
    release, samples = record_from_release(rec, args, record_s, during=publish)
    flags = [m.data for _, m in rec.received[flag_topic]]
    rec.destroy_node()
    if release is None:
        return finish(args.out, {}, COULD_NOT_JUDGE,
                      f"no 'releasing the simulated robot' line in the launch log within {args.wait} s")
    write_samples(args.out, samples)
    if state["step_wall_s"] is None:
        return finish(args.out, {"samples": len(samples)}, COULD_NOT_JUDGE, "the step never started")
    step_t = state["step_wall_s"] - release  # [s after release] the first step command left
    after = [s for s in samples if s.t_s >= step_t]
    target = args.rise_fraction * args.speed_m_s
    rise_at = next((s.t_s for s in after if s.mean_velocity_m_s >= target), None)
    rise_s = rise_at - step_t if rise_at is not None else None
    # Overshoot is judged on the replica's velocity with each cycle's real length (see
    # velocity_from_true_periods); the highest exported sample is kept for the record.
    corrected, replay_problem, sample_gaps = (velocity_from_true_periods(samples) if samples
                                              else ([], "no samples", 0))
    peak_v = max((v for s, v in zip(samples, corrected) if s.t_s >= step_t), default=math.nan)
    overshoot_pct = 100.0 * (peak_v - args.speed_m_s) / args.speed_m_s
    peak_exported_v = max((s.mean_velocity_m_s for s in after), default=math.nan)
    exported_overshoot_pct = 100.0 * (peak_exported_v - args.speed_m_s) / args.speed_m_s
    peak_pitch = max((abs(s.pitch_deg) for s in samples), default=math.nan)
    fell = fell_at(samples, args.tipover_deg)
    metrics = {
        "samples": len(samples), "release_stamp_s": release, "gamepad_messages": state["published"],
        "timed_out_flags_seen": flags, "step_at_s": step_t, "rise_time_s": rise_s,
        "peak_mean_velocity_m_s": peak_v, "overshoot_pct": overshoot_pct,
        "peak_exported_mean_velocity_m_s": peak_exported_v,
        "exported_overshoot_pct": exported_overshoot_pct, "replay_problem": replay_problem,
        "sample_pairs_with_dropped_samples": sample_gaps,
        "min_mean_velocity_after_step_m_s": min((s.mean_velocity_m_s for s in after), default=math.nan),
        "final_mean_velocity_m_s": samples[-1].mean_velocity_m_s if samples else None,
        "peak_abs_pitch_deg": peak_pitch, "fell_at_s": fell,
        "speed_m_s": args.speed_m_s, "rise_fraction": args.rise_fraction,
        "rise_within_s": args.rise_within_s, "max_overshoot_pct": args.max_overshoot_pct,
        "max_pitch_deg": args.max_pitch_deg, "tipover_deg": args.tipover_deg,
        "last_stamp_s": release + samples[-1].t_s if samples else None,
    }
    print(f"scenario_checks: step to {args.speed_m_s} m/s at {step_t:.3f} s after release; "
          f"{args.rise_fraction:.0%} reached after {rise_s if rise_s is None else round(rise_s, 3)} s; "
          f"peak mean v {peak_v:.4f} m/s with true cycle lengths (overshoot {overshoot_pct:.1f} %; "
          f"highest exported sample {peak_exported_v:.4f} m/s, {exported_overshoot_pct:.1f} %), lowest "
          f"{metrics['min_mean_velocity_after_step_m_s']:.4f} m/s; peak |pitch| {peak_pitch:.3f} deg; "
          f"|pitch| >= {args.tipover_deg} deg at {fell if fell is None else round(fell, 3)} s")
    if False in flags and True in flags[flags.index(False) + 1:]:
        print("scenario_checks: note: command_timed_out went True while the gamepad was driving "
              "(a delivery gap longer than the watchdog's timeout)")
    if fell is not None:
        return finish(args.out, metrics, FAIL,
                      f"the pitch diverged: |pitch| reached {args.tipover_deg} deg {fell:.3f} s after "
                      f"release ({fell - step_t:.3f} s after the step; the robot fell)")
    problem = coverage_problem(samples, args, record_s) or replay_problem
    if problem:
        return finish(args.out, metrics, COULD_NOT_JUDGE, problem)
    if rise_s is None or rise_s > args.rise_within_s:
        return finish(args.out, metrics, FAIL,
                      f"the mean measured velocity did not reach {target:g} m/s within "
                      f"{args.rise_within_s} s of the step (reached after {rise_s} s)")
    if overshoot_pct > args.max_overshoot_pct:
        return finish(args.out, metrics, FAIL,
                      f"overshoot {overshoot_pct:.1f} % (limit {args.max_overshoot_pct} %; highest "
                      f"exported sample {exported_overshoot_pct:.1f} %)")
    if peak_pitch > args.max_pitch_deg:
        return finish(args.out, metrics, FAIL,
                      f"|pitch| reached {peak_pitch:.3f} deg (limit {args.max_pitch_deg} deg)")
    return finish(args.out, metrics, PASS,
                  f"followed the step: {args.rise_fraction:.0%} in {rise_s:.3f} s, overshoot "
                  f"{overshoot_pct:.1f} % (highest exported sample {exported_overshoot_pct:.1f} %), "
                  f"peak |pitch| {peak_pitch:.3f} deg")


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
    p = add("feedforward-sends", check_feedforward_sends, "seconds to record")
    p.add_argument("--gap-s", type=float, required=True,
                   help="report send gaps longer than this [s] (FF_AGE_S)")
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
