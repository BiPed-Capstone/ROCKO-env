#!/usr/bin/env python3
"""Record one run of the robot: a rosbag (MCAP) plus a small file that says what was run.

Started by the launch files when given record:=true, and by the web dashboard's Record button
(webserver/Dashboard.py). By hand, on the machine running the robot code, in a shell with the workspace sourced:

    ros2 run rocko_env RecordRun.py --label pitch_step --battery-v 12.1 --surface tile

It makes   <log root>/<YYYYMMDD_HHMMSS>_<label>/
               run.yaml   what was run: commit, launch, config, battery, surface, topics
               bag/       the rosbag2 recording (MCAP: Foxglove opens the .mcap file directly)
and records until it is stopped with Ctrl-C (or the launch that started it ends).

Why the extra file: a recording without its conditions cannot be compared with another one
(docs/guides/reproducing-results.md). The motor command is open loop, so battery voltage is part
of the plant; nobody remembers it a week later.

This is a plain program, not a ROS node. It is NOT on the 100 Hz control path: it runs as its own
process and only starts `ros2 bag record`, which subscribes like any other listener.
"""
import argparse
import hashlib
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

DEFAULT_LOG_ROOT = "~/rocko_logs"

# Recording is refused, and a running recording is stopped, when the disk holding the log root has
# less than this free [MB]. A full SD card on the Pi takes the whole robot down, not just the log.
DEFAULT_MIN_FREE_MB = 500

# How often the free space is looked at while recording [s].
DISK_CHECK_PERIOD_S = 5.0

# What is recorded. ONE list, here, so every way of starting a recording records the same thing.
# Edit this list to change what is recorded; run.yaml copies it.
# A topic that does not exist in the running mode is simply never written (tank mode has no PID
# controllers; the simulation has no /joystick until a sender connects, and no /joy at all).
RECORDED_TOPICS = [
    # The robot's state, as the controllers see it: pitch [deg], pitch_rate [deg/s], wheel position
    # and velocity, and the wheel velocity commands. 100 Hz.
    "/dynamic_joint_states",
    "/joint_states",
    # Each PID loop's reference, feedback, error and output.
    "/pitch_pid_controller/controller_state",
    "/velocity_pid_controller/controller_state",
    # What the operator asked for, and what DiffDriveController made of it. /joystick is balancing
    # mode's gamepad input; /joy and /tank_teleop/enabled are tank mode's USB controller and its
    # hold-LB-to-drive state.
    "/joystick",
    "/joy",
    "/tank_teleop/enabled",
    "/diffbot_base_controller/cmd_vel",
    "/velocity_pid_controller/reference",
    "/left_feedforward",
    "/right_feedforward",
    # The safeguards: teleop watchdog state, tipover re-arm requests.
    "/diff_drive_controller/command_timed_out",
    "/tipover_rearm",
    # Every node's log lines (the tipover and time-out messages are here), controller activity,
    # and how long each control cycle took.
    "/rosout",
    "/controller_manager/activity",
    "/controller_manager/statistics/full",
    "/diagnostics",
    # Tank mode's odometry, and what a viewer needs to draw the robot.
    "/diffbot_base_controller/odom",
    "/robot_description",
    "/tf",
    "/tf_static",
]

NOT_RECORDED = "not recorded"  # written for a field nobody filled in: a blank looks like a forgotten one


def free_mb(path):
    """Free space [MB] on the disk that holds `path`."""
    return shutil.disk_usage(path).free / 1e6


def git_commit(repo_dir):
    """(short commit hash, "clean" | "dirty" | "unknown") of the checkout the code runs from."""
    def git(*args):
        return subprocess.run(["git", "-C", repo_dir, *args], capture_output=True, text=True, timeout=5)
    try:
        commit = git("rev-parse", "--short", "HEAD")
        if commit.returncode != 0:
            return "unknown", "unknown"
        status = git("status", "--porcelain")
        tree = "unknown" if status.returncode != 0 else ("dirty" if status.stdout.strip() else "clean")
        return commit.stdout.strip(), tree
    except (OSError, subprocess.SubprocessError):
        return "unknown", "unknown"


def sha256_of(path):
    """The file's SHA-256, or "unknown" if it cannot be read. Says exactly which gains were live."""
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return "unknown"


def safe_label(label):
    """The label as a folder-name part: letters, digits, '-' and '_' kept, anything else '_'."""
    cleaned = "".join(c if c.isalnum() or c in "-_" else "_" for c in label.strip())
    return cleaned[:60] or "run"


def run_metadata(args, started):
    """The fields of run.yaml, in the order they are written."""
    # This file is run in place (colcon --symlink-install), so its real path is inside the checkout.
    repo_dir = os.path.dirname(os.path.realpath(__file__))
    commit, tree = git_commit(repo_dir)
    return [
        ("label", args.label),
        ("started", time.strftime("%Y-%m-%d %H:%M:%S %z", started)),
        ("host", socket.gethostname()),
        ("commit", commit),
        ("tree", tree),
        ("launch", args.launch or NOT_RECORDED),
        ("hardware", args.hardware or NOT_RECORDED),
        ("config", args.config or NOT_RECORDED),
        ("config_sha256", sha256_of(args.config) if args.config else NOT_RECORDED),
        ("battery_v", args.battery_v or NOT_RECORDED),
        ("surface", args.surface or NOT_RECORDED),
        ("notes", args.notes or NOT_RECORDED),
        ("topics", RECORDED_TOPICS),
    ]


def write_run_yaml(path, fields):
    """Write the fields as YAML. json.dumps gives a quoted string, which is also valid YAML."""
    with open(path, "w") as f:
        f.write("# What this recording is of. Written by RecordRun.py when the recording started.\n")
        for key, value in fields:
            if isinstance(value, list):
                f.write(f"{key}:\n")
                for item in value:
                    f.write(f"  - {json.dumps(item)}\n")
            else:
                f.write(f"{key}: {json.dumps(value)}\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--label", default="run", help="a short name for the test, e.g. pitch_step")
    parser.add_argument("--log-root", default="",
                        help=f"folder the run folder is made in (empty = {DEFAULT_LOG_ROOT})")
    parser.add_argument("--min-free-mb", type=float, default=DEFAULT_MIN_FREE_MB,
                        help="refuse to record, or stop, below this much free disk [MB]")
    parser.add_argument("--launch", default="", help="the launch command that is running, with its arguments")
    parser.add_argument("--hardware", default="", help="real, mock or sim")
    parser.add_argument("--config", default="", help="path of the controllers YAML in use")
    parser.add_argument("--battery-v", default="", help="battery voltage at the start [V]")
    parser.add_argument("--surface", default="", help="what the robot stands on: carpet, tile, a stand ...")
    parser.add_argument("--notes", default="", help="anything else worth knowing later")
    args, _ = parser.parse_known_args()  # a launch file adds --ros-args ...; this program ignores them

    log_root = os.path.expanduser(args.log_root or DEFAULT_LOG_ROOT)
    os.makedirs(log_root, exist_ok=True)
    if free_mb(log_root) < args.min_free_mb:
        print(f"RecordRun: NOT recording: {free_mb(log_root):.0f} MB free in {log_root}, "
              f"need {args.min_free_mb:.0f} MB. Copy old runs off (tools/fetch_logs.sh) and delete them.",
              file=sys.stderr, flush=True)
        return 1

    started = time.localtime()
    run_dir = os.path.join(log_root, time.strftime("%Y%m%d_%H%M%S", started) + "_" + safe_label(args.label))
    os.makedirs(run_dir)  # fails if it exists: two recordings never share a folder
    write_run_yaml(os.path.join(run_dir, "run.yaml"), run_metadata(args, started))

    # start_new_session: the recorder is stopped by US (below), once, not also by the terminal's
    # Ctrl-C, so the bag is always closed the same way.
    stop_reason = None   # why the recording ended, for run.yaml and the log
    failed = False       # True if it did not end the way it should (decides the exit status)

    def ask_to_stop(signum, _frame):
        nonlocal stop_reason
        stop_reason = stop_reason or f"signal {signal.Signals(signum).name}"

    signal.signal(signal.SIGINT, ask_to_stop)
    signal.signal(signal.SIGTERM, ask_to_stop)

    # The handlers above are installed BEFORE the recorder is started: a stop that arrived in
    # between would end this program with the recorder still running and nobody left to stop it.
    # preexec_fn: make sure the recorder LISTENS to SIGINT. A program started in the background by a
    # script (or by some service managers) inherits "ignore SIGINT", and would then never close
    # the bag when asked: seen in verify_tools.sh, where the bag had no metadata.yaml.
    recorder = subprocess.Popen(
        ["ros2", "bag", "record", "--storage", "mcap", "--output", os.path.join(run_dir, "bag"),
         *RECORDED_TOPICS],
        start_new_session=True,
        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
    print(f"RecordRun: recording to {run_dir}", flush=True)

    last_disk_check_s = time.monotonic()
    while recorder.poll() is None and stop_reason is None:
        time.sleep(0.2)
        if time.monotonic() - last_disk_check_s >= DISK_CHECK_PERIOD_S:
            last_disk_check_s = time.monotonic()
            if free_mb(log_root) < args.min_free_mb:
                stop_reason = f"only {free_mb(log_root):.0f} MB free"
                failed = True

    if recorder.poll() is not None:
        stop_reason = f"ros2 bag record exited by itself with status {recorder.returncode}"
        failed = True
    else:
        # SIGINT is how `ros2 bag record` is asked to finish: it closes the file and writes its
        # metadata.yaml. Killing it outright would leave a bag some tools refuse to open.
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=15)
        except subprocess.TimeoutExpired:
            recorder.kill()
            stop_reason += "; the recorder did not close in 15 s and was killed"
            failed = True
    with open(os.path.join(run_dir, "run.yaml"), "a") as f:
        f.write(f"stopped: {json.dumps(time.strftime('%Y-%m-%d %H:%M:%S %z'))}\n")
        f.write(f"stop_reason: {json.dumps(stop_reason)}\n")
    print(f"RecordRun: stopped ({stop_reason}). The run is in {run_dir}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
