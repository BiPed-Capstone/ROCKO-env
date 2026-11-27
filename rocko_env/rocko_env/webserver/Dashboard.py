#!/usr/bin/env python3
"""The robot's web dashboard: live values in a browser on another machine, and Record / Stop.

Started by tank mode and the simulation by default, and by rocko.launch.py with dashboard:=true. Then, on any machine on the same network, open

    http://<robot-ip>:8080

What it is: one ROS node that listens to a handful of topics, keeps the newest values and the last
HISTORY_S seconds of them, and a small web server (Python's standard library: no Flask, nothing to
install) that hands them to the page as JSON. The page is one file, dashboard.html, with no outside
scripts, so it works on a network with no internet.

What it can change: (1) it can start and stop a recording (it runs RecordRun.py, the same program
record:=true runs) and hand recorded runs, or single files of them, to the browser. The status screen's
record button asks for a recording through this node's service /rocko/record (std_srvs/SetBool: true =
record for record_button_s, then stop by itself; false = stop), so there is one recorder whoever asks;
(2) it can ASK the robot supervisor (Supervisor.py, docs/architecture/robot-supervisor.md) to enable or disable the motion controllers
and to switch between tank and balance mode, through the services /rocko/set_enabled and
/rocko/set_mode; (3) it can ask the buzzer to mute or unmute (/rocko/set_buzzer_muted). That is ALL.
It calls no other service, and its only publication is its recording report ("rocko/recording" on
/diagnostics, once a second: whether a recording runs, its folder, the time left, where runs are saved and
the free space), so it cannot drive the robot, re-arm the tipover cutoff or touch a controller itself: the supervisor decides, and
refuses what its rules forbid (for example a mode change while enabled). It shows the supervisor's
state (/rocko/state) at the top of the page, and what each optional extra (battery monitor, buzzer,
status screen) reports on /diagnostics (docs/architecture/robot-extras.md).

It is NOT on the 100 Hz control path: it is a separate process, and it lowers its own priority
(NICENESS) so the control loop is served first. What it costs the Pi's CPU is not measured.

There is no login. "No login" means anyone on the robot's network can see the page and press Record,
Enable, Disable, Tank or Balance: Enable makes the motion controllers active, so a stick can then move
the wheels. The only check is that a POST comes from this page and not from another website
(is_own_address). Do not run it on a network you do not trust. Closing the page does NOT disable
the robot: the dashboard is not a deadman.
"""
import errno
import ipaddress
import json
import math
import os
import signal
import socket
import shutil
import subprocess
import tarfile
import threading
import time
import urllib.parse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
import rclpy.executors
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState, MultiDOFCommand, MultiDOFStateStamped
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rocko_interfaces.msg import RobotState
from rocko_interfaces.srv import SetEnabled, SetMode
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import BatteryState, Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# The folder the recorder writes is the folder the run list reads: one constant, RecordRun.py's.
from rocko_env.recording.RecordRun import DEFAULT_LOG_ROOT
# The optional extras' reports, and the tipover cutoff's log phrases (shared with the screen and the buzzer).
from rocko_env.extras import health, log_phrases, sysinfo
# The controller's layout (which /joy index is which stick or button): the shared teleop module.
from rocko_env.teleop import gamepad

DEFAULT_PORT = 8080
SAMPLE_PERIOD_S = 0.05   # [s] the history gets one point this often: 20 per second is plenty for a
                         #   plot a person watches, and a fifth of the 100 Hz the robot produces.
                         #   The battery plot's title in dashboard.html says "50 ms": change both
HISTORY_S = 60.0         # [s] how far back the page's plots can go
NICENESS = 10            # added to this process's "nice" value: higher = lower priority (0 = normal)
STALE_AFTER_S = 1.0      # [s] a value older than this is shown as stale: its publisher has gone quiet
LOG_LINES_KEPT = 40      # newest warnings and errors kept for the page
SUPERVISOR_STALE_S = 2.0   # [s] no /rocko/state for this long = "no supervisor" (it publishes at 2 Hz)
SERVICE_TIMEOUT_S = 3.0    # [s] how long a button press waits for the supervisor's answer
SOURCE = "dashboard"       # the `source` field of every request: shows in /rocko/state's last_source
RECORD_BUTTON_S = 60.0     # [s] how long the status screen's record button records (parameter record_button_s)
SCREEN_LABEL = "screen"    # the label of a run the screen's button started: it ends the run folder's name
STOP_WAIT_S = 25.0         # [s] how long a stop waits for the recorder to close its bag
WHEELS = ("left_wheel_joint", "right_wheel_joint")


class Dashboard(Node):
    def __init__(self):
        super().__init__("rocko_dashboard")
        self.port = self.declare_parameter("port", DEFAULT_PORT).value
        # Empty (the launch files' default) means DEFAULT_LOG_ROOT, as it does for RecordRun.py.
        self.log_root = os.path.expanduser(self.declare_parameter("log_root", "").value or DEFAULT_LOG_ROOT)
        # What RecordRun.py should write into run.yaml about this launch (the launch file fills these in).
        self.launch_name = self.declare_parameter("launch", "").value
        self.hardware = self.declare_parameter("hardware", "").value
        self.config = self.declare_parameter("config", "").value
        # Read at each button press (not only here), so a check can shorten it with `ros2 param set`.
        self.declare_parameter("record_button_s", RECORD_BUTTON_S)

        # Everything the web threads read is guarded by this lock; ROS callbacks write under it.
        self.lock = threading.Lock()
        self.latest = {}        # signal name -> newest value. The names (pitch_deg, ...) are the
                                #   link to the page: TILES and PLOTS in dashboard.html use them
        self.updated_s = {}     # topic group -> time.monotonic() of its newest message
        self.history = deque(maxlen=int(HISTORY_S / SAMPLE_PERIOD_S))  # (unix time, {signal: value})
        self.log_lines = deque(maxlen=LOG_LINES_KEPT)  # {"t", "level", "node", "text"}
        self.tipover_by_node = {}  # controller node name -> True (tripped) / False (re-armed), from its log
        self.controller = None  # the newest /joy as {"axes": {name: value}, "buttons": {name: 0/1}}, or None
        self.robot = None       # the newest /rocko/state as a dict (see on_robot_state), or None
        self.robot_at_s = None  # time.monotonic() when it arrived
        self.recorder = None    # the running RecordRun.py process, or None
        self.recording_label = ""
        self.record_until_s = None  # [monotonic s] when a timed recording (the screen's button) stops itself, or None
        self.runs_before = set()    # the run folders that existed when the recording started: the new one is its own
        self.stopping = False       # a stop is under way in its own thread
        # Held for the whole of a stop (up to STOP_WAIT_S), so a second stop waits for the first instead of signalling
        # the recorder again while it closes its bag. Not self.lock: that one must never be held while waiting.
        self.stop_lock = threading.Lock()
        self.extra_reports = {}  # extra -> its newest health.report(), from /diagnostics
        self.extra_report_s = {} # extra -> time.monotonic() when it arrived
        self.battery_low_v = None  # the lowest battery reading since the last sample() [V], or None

        # The fast topics (100 Hz) are subscribed RAW: the callback only keeps the newest message's
        # bytes, and sample() decodes that one message 20 times a second. Decoding every message in
        # Python took more time than there was: the first version fell behind and its history got
        # 6 points a second instead of 20 (seen in verify_tools.sh). It is also most of what this
        # node would cost the Pi.
        # depth 1: only the newest message matters, and a short queue means a slow moment here
        # drops old messages instead of making the publisher wait.
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # topic group -> (message type, function that turns a decoded message into {signal: value})
        self.fast_topics = {
            "joint_states": ("/dynamic_joint_states", DynamicJointState, self.joint_state_values),
            "pitch_pid": ("/pitch_pid_controller/controller_state", MultiDOFStateStamped, self.pitch_pid_values),
            "velocity_pid": ("/velocity_pid_controller/controller_state", MultiDOFStateStamped,
                             self.velocity_pid_values),
            "reference": ("/velocity_pid_controller/reference", MultiDOFCommand, self.reference_values),
        }
        self.raw = {}  # topic group -> the newest message's bytes, not yet decoded (guarded by self.lock)
        for group, (topic, msg_type, _) in self.fast_topics.items():
            self.create_subscription(
                msg_type, topic, lambda data, g=group: self.keep_raw(g, data), newest, raw=True)
        self.create_subscription(
            Bool, "/diff_drive_controller/command_timed_out", self.on_command_timed_out, latched)
        self.create_subscription(Log, "/rosout", self.on_log, 20)
        # Tank mode's game controller (joy's game_controller_node, 20 messages a second) and whether
        # TankTeleop.py lets it drive. Slow enough to decode in the callback. Neither exists in
        # balancing mode or the simulation; the page then says "no controller data".
        self.create_subscription(Joy, "/joy", self.on_joy, 5)
        self.create_subscription(Bool, "/tank_teleop/enabled", self.on_teleop_enabled, 5)
        # The supervisor's state: reliable + transient-local depth 1, so a dashboard that starts late
        # still gets the last state at once. (Must match the publisher's QoS or nothing arrives.)
        self.create_subscription(
            RobotState, "/rocko/state", self.on_robot_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.set_enabled_client = self.create_client(SetEnabled, "/rocko/set_enabled")
        self.set_mode_client = self.create_client(SetMode, "/rocko/set_mode")
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.mute_client = self.create_client(SetBool, "/rocko/set_buzzer_muted")
        self.create_service(SetBool, "/rocko/record", self.on_record_request)
        self.recording_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_timer(health.REPORT_PERIOD_S, self.recording_tick)
        # Every battery reading (50 Hz, BatteryMonitor.py), so the plot can show the LOWEST one in each sample: a
        # brown-out shorter than SAMPLE_PERIOD_S would be missed by plotting only the newest. Decoded here: one
        # small message 50 times a second. Absent when the battery monitor is not fitted.
        self.create_subscription(BatteryState, "/rocko/battery_raw", self.on_battery_raw,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(SAMPLE_PERIOD_S, self.sample)

    # --- ROS callbacks: keep bytes, or copy a number; nothing slow ---------------------------------
    def keep_raw(self, group, data):
        with self.lock:
            self.raw[group] = data
            self.updated_s[group] = time.monotonic()

    def set_values(self, group, values):
        with self.lock:
            self.latest.update(values)
            self.updated_s[group] = time.monotonic()

    # --- decoded message -> {signal name: value}; called from sample(), 20 times a second ---------
    # Each name ends in its unit. SIGNS are passed on as the robot reports them and are NOT verified
    # on the robot: which way positive pitch leans, and whether a positive wheel velocity is
    # forward, are open questions (docs/architecture/sim-plant.md section 2.4, proposal 0006).
    # To show one more signal: return it from one of these functions under a new name, then add
    # that name to TILES (and, for a plot, PLOTS) in dashboard.html. The spelling must match.
    @staticmethod
    def joint_state_values(msg):
        values = {}
        for joint, interfaces in zip(msg.joint_names, msg.interface_values):
            if joint not in WHEELS:
                continue
            side = joint.split("_")[0]  # "left" or "right"
            for name, value in zip(interfaces.interface_names, interfaces.values):
                if name == "velocity":
                    values[f"{side}_velocity_m_s"] = value   # measured [m/s at the ground], as the
                                                             #   encoder node reports it
                elif joint == WHEELS[0] and name == "pitch":
                    values["pitch_deg"] = value              # the IMU plugin exports it on both joints
                elif joint == WHEELS[0] and name == "pitch_rate":
                    values["pitch_rate_deg_s"] = value
        return values

    @staticmethod
    def pitch_pid_values(msg):
        if not msg.dof_states:
            return {}
        state = msg.dof_states[0]  # both wheels' entries carry the same pitch
        # The pitch loop's output is the wheels' ANGULAR velocity command [rad/s], which is why it
        # is not in m/s like the measured speeds above (speed = rad/s x wheel radius).
        return {"target_pitch_deg": state.reference,     # what the velocity loop asked for
                "wheel_command_rad_s": state.output}     # before the feedforward is added

    @staticmethod
    def velocity_pid_values(msg):
        return {"velocity_error_m_s": msg.dof_states[0].error} if msg.dof_states else {}

    @staticmethod
    def reference_values(msg):
        return {"velocity_reference_m_s": msg.values[0]} if msg.values else {}

    def on_command_timed_out(self, msg):
        self.set_values("watchdog", {"command_timed_out": bool(msg.data)})

    def on_joy(self, msg):
        # By NAME (gamepad.AXIS_NAMES / BUTTON_NAMES), so the page never needs the index numbers.
        # A shorter message (another node's layout) simply has fewer names.
        # A non-finite axis becomes None: NaN is not valid JSON, and the browser would reject the reply.
        controller = {"axes": dict(zip(gamepad.AXIS_NAMES,
                                       (float(v) if math.isfinite(v) else None for v in msg.axes))),
                      "buttons": dict(zip(gamepad.BUTTON_NAMES, (int(v) for v in msg.buttons)))}
        with self.lock:
            self.controller = controller
            self.updated_s["controller"] = time.monotonic()

    def on_teleop_enabled(self, msg):
        self.set_values("teleop_enabled", {"teleop_enabled": bool(msg.data)})

    def on_robot_state(self, msg):
        robot = {
            "mode": msg.mode, "enabled": bool(msg.enabled), "phase": msg.phase,
            "requested_mode": msg.requested_mode, "last_source": msg.last_source, "message": msg.message,
            "available_modes": list(msg.available_modes),
            "nodes": [{"name": n.name, "group": n.group, "alive": bool(n.alive)} for n in msg.nodes],
            "controllers": [{"name": c.name, "state": c.state, "expected_state": c.expected_state}
                            for c in msg.controllers],
        }
        with self.lock:
            self.robot = robot
            self.robot_at_s = time.monotonic()

    def robot_snapshot(self):
        """The supervisor's state for the page: {"present": False, ...} when none was heard in
        SUPERVISOR_STALE_S (a launch started by hand has no supervisor; so does a dead one)."""
        with self.lock:
            robot, at = self.robot, self.robot_at_s
        age = None if at is None else time.monotonic() - at
        if robot is None or age > SUPERVISOR_STALE_S:
            # The old values are not repeated: a stale "enabled: false" would be a safe-looking guess.
            return {"present": False, "age_s": age, "message": "no supervisor"}
        return {"present": True, "age_s": age, **robot}

    def on_log(self, msg):
        if msg.level < Log.WARN:
            return
        level = {Log.WARN: "WARN", Log.ERROR: "ERROR", Log.FATAL: "FATAL"}.get(msg.level, "WARN")
        with self.lock:
            self.log_lines.append({"t": time.time(), "level": level, "node": msg.name, "text": msg.msg})
            # The phrases are AddFeedforwardController.cpp's log lines (log_phrases.py).
            # Each wheel's controller trips and re-arms on its own, so the state is kept per node
            # and the page shows TRIPPED while ANY of them is. Until a line has been seen there is
            # no entry at all, and the page says so: a trip logged before this node started cannot
            # be known here, and "armed" would be a safe-looking guess (the mistake of finding C-2).
            change = log_phrases.tipover_change(msg.msg)
            if change is not None:
                self.tipover_by_node[msg.name] = change
            if self.tipover_by_node:
                self.latest["tipover_tripped"] = any(self.tipover_by_node.values())

    def on_battery_raw(self, msg):
        with self.lock:
            low = self.battery_low_v
            self.battery_low_v = msg.voltage if low is None else min(low, msg.voltage)
            self.updated_s["battery"] = time.monotonic()

    def on_diagnostics(self, msg):
        now = time.monotonic()
        with self.lock:
            for extra, status in health.from_message(msg).items():
                self.extra_reports[extra], self.extra_report_s[extra] = status, now

    def sample(self):
        """Every SAMPLE_PERIOD_S: decode the newest message of each fast topic, then add one point
        of history, numbers only (a NaN is left out, so the plot shows a gap)."""
        if not rclpy.ok():
            return  # shutting down: ROS can no longer decode a message (it raised a TypeError here)
        with self.lock:
            raw, self.raw = self.raw, {}
        for group, data in raw.items():
            _, msg_type, to_values = self.fast_topics[group]
            values = to_values(deserialize_message(data, msg_type))
            with self.lock:
                self.latest.update(values)
        with self.lock:
            if self.battery_low_v is not None:
                self.latest["battery_low_v"] = self.battery_low_v
                self.battery_low_v = None   # start the next sample's lowest reading afresh
            else:
                # No reading in this sample (a skipped read during a brown-out, or no monitor): a gap in the plot,
                # never the previous value, which would draw a flat line straight through the sag.
                self.latest.pop("battery_low_v", None)
            point = {key: value for key, value in self.latest.items()
                     if isinstance(value, float) and math.isfinite(value)}
            self.history.append((time.time(), point))

    # --- what the web server reads --------------------------------------------------------------
    def state(self):
        now = time.monotonic()
        with self.lock:
            latest = {key: (value if not isinstance(value, float) or math.isfinite(value) else None)
                      for key, value in self.latest.items()}
            ages = {group: now - t for group, t in self.updated_s.items()}
            logs = list(self.log_lines)
            controller = self.controller
            extras = health.summary(self.extra_reports, {k: now - t for k, t in self.extra_report_s.items()})
        return {
            "controller": controller,
            "time": time.time(), "values": latest, "age_s": ages, "stale_after_s": STALE_AFTER_S,
            "logs": logs, "recording": self.recording_status(),
            "launch": self.launch_name, "hardware": self.hardware,
            "robot": self.robot_snapshot(),
            "extras": extras,
        }

    def history_since(self, since):
        with self.lock:
            return [{"t": t, **point} for t, point in self.history if t > since]

    # --- asking the supervisor ---------------------------------------------------------------------
    def call_supervisor(self, client, request, who="supervisor"):
        """Send `request` to a supervisor service (or `who` else's) and wait up to SERVICE_TIMEOUT_S for the answer.
        Returns (success, message).

        Why waiting here does not block ROS: this runs in an HTTP handler thread (ThreadingHTTPServer
        gives each request its own), while main() runs rclpy.spin() in the main thread. The executor
        keeps spinning, so it delivers the supervisor's reply and runs the future's done-callback,
        which sets the Event this thread waits on. Waiting with future.result() or spin_until_future_
        complete() from here would be wrong: the first can hang if the executor is busy, the second
        would start a second executor on the same node."""
        if not client.service_is_ready():
            return False, f"{who} did not answer (service not available)"
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(SERVICE_TIMEOUT_S):
            client.remove_pending_request(future)  # forget it, so a late answer is not kept forever
            return False, f"{who} did not answer"
        try:
            response = future.result()
        except Exception as error:  # the call failed (for example the supervisor went away)
            return False, f"{who} call failed: {error}"
        return bool(response.success), response.message

    def set_enabled(self, enabled):
        request = SetEnabled.Request()
        request.enabled = enabled
        request.source = SOURCE
        return self.call_supervisor(self.set_enabled_client, request)

    def set_mode(self, mode):
        request = SetMode.Request()
        request.mode = mode
        request.source = SOURCE
        return self.call_supervisor(self.set_mode_client, request)

    def set_muted(self, muted):
        return self.call_supervisor(self.mute_client, SetBool.Request(data=muted), who="buzzer")

    def launch_label(self):
        """launch_name, plus the mode when the supervisor is heard, so run.yaml says which stack ran
        (rocko_robot.launch.py starts whichever mode's launch file as a child)."""
        robot = self.robot_snapshot()
        return f"{self.launch_name} (mode {robot['mode']})" if robot["present"] else self.launch_name

    # --- recording: start and stop RecordRun.py --------------------------------------------------
    def recording_status(self):
        """Whether a recording runs, and what the page and the screen show about it. folder: the new run's folder
        name while recording ("" before RecordRun.py has made it); seconds_left: None when it has no end."""
        with self.lock:
            running = self.recorder is not None and self.recorder.poll() is None
            label, until, before = self.recording_label, self.record_until_s, self.runs_before
        disk = sysinfo.read_disk_gb(self.log_root) if os.path.isdir(self.log_root) else None
        new = sorted(set(self.run_names()) - before) if running else []
        return {"running": running, "label": label if running else "",
                "folder": new[-1] if new else "",
                "seconds_left": max(0.0, until - time.monotonic()) if running and until is not None else None,
                "log_root": self.log_root,
                "free_gb": None if disk is None else disk["free_gb"],
                "total_gb": None if disk is None else disk["total_gb"],
                "low_space": disk is not None and sysinfo.disk_low(disk["free_gb"])}

    def battery_now(self):
        """The battery monitor's smoothed voltage as text, for run.yaml, or "" when it has not reported lately."""
        with self.lock:
            status, at = self.extra_reports.get("battery"), self.extra_report_s.get("battery")
        if status is None or at is None or time.monotonic() - at > health.STALE_AFTER_S:
            return ""
        return status["values"].get("voltage", "")

    def on_record_request(self, request, response):
        """/rocko/record, for the status screen's button: true = record for record_button_s, false = stop.
        Runs in the ROS executor, so it must not wait: starting only launches a process, and a stop (which waits up
        to STOP_WAIT_S for the bag to close) runs in its own thread."""
        if request.data:
            seconds = float(self.get_parameter("record_button_s").value)
            response.success, response.message = self.start_recording(
                SCREEN_LABEL, self.battery_now(), "", notes="started from the status screen", duration_s=seconds)
        else:
            response.success, response.message = self.stop_in_background()
        return response

    def stop_in_background(self):
        with self.lock:
            if self.stopping:
                return True, "already stopping"
            if self.recorder is None or self.recorder.poll() is not None:
                return False, "no recording is running"
            self.stopping = True

        def stop():
            try:
                self.stop_recording()
            finally:
                with self.lock:
                    self.stopping = False
        threading.Thread(target=stop, daemon=True).start()
        return True, "stopping"

    def recording_tick(self):
        """Once a second: stop a timed recording whose time is up, and publish the recording report."""
        with self.lock:
            due = (self.record_until_s is not None and time.monotonic() >= self.record_until_s
                   and self.recorder is not None and self.recorder.poll() is None)
        if due:
            self.stop_in_background()
        status = self.recording_status()
        left = status["seconds_left"]
        if status["running"]:
            message = f"recording {status['folder'] or '(starting)'}" + (f", {left:.0f} s left" if left is not None else "")
        else:
            message = "not recording"
        level = health.WARN if status["low_space"] else health.OK
        self.recording_pub.publish(health.to_message(health.report("recording", level, message, {
            "running": "true" if status["running"] else "false",
            "seconds_left": "" if left is None else f"{left:.1f}",
            "folder": status["folder"],
            "log_root": status["log_root"],
            "free_gb": "" if status["free_gb"] is None else f"{status['free_gb']:.2f}",
            "low_space": "true" if status["low_space"] else "false",
            "port": self.port,
            "button_s": f"{self.get_parameter('record_button_s').value:g}",
        })))

    def start_recording(self, label, battery_v, surface, notes="started from the dashboard", duration_s=None):
        # Taken BEFORE the lock: launch_label() -> robot_snapshot() takes self.lock itself, and a
        # threading.Lock is not re-entrant. Asking for it while holding it froze this HTTP thread, then
        # every ROS callback and /api/state behind it, and made SIGINT hang (verify_tools.sh, 2026-11-23).
        launch_label = self.launch_label()
        with self.lock:
            if self.recorder is not None and self.recorder.poll() is None:
                return False, "a recording is already running"
            self.runs_before = set(self.run_names())
            # start_new_session: the launch's Ctrl-C must not reach RecordRun.py twice (once from
            # the terminal, once from stop_recording below).
            self.recorder = subprocess.Popen(
                ["ros2", "run", "rocko_env", "RecordRun.py", "--label", label, "--log-root", self.log_root,
                 "--launch", launch_label, "--hardware", self.hardware, "--config", self.config,
                 "--battery-v", battery_v, "--surface", surface, "--notes", notes],
                start_new_session=True)
            self.recording_label = label
            self.record_until_s = None if duration_s is None else time.monotonic() + duration_s
        return True, "recording" + ("" if duration_s is None else f" for {duration_s:g} s")

    def stop_recording(self):
        with self.lock:
            recorder = self.recorder
        if recorder is None or recorder.poll() is not None:
            return False, "no recording is running"
        # One stop at a time (the page's Stop while the screen's timed stop is closing the bag, or a shutdown): a
        # second caller waits here, then finds the recorder gone and reports it stopped.
        with self.stop_lock:
            if recorder.poll() is not None:
                return True, "stopped"
            # killpg signals every process in the recorder's group: `ros2 run` AND RecordRun.py, which
            # closes the bag. (`ros2 run` alone would not pass the signal on; see launch_recording.py.)
            try:
                os.killpg(recorder.pid, signal.SIGINT)
            except ProcessLookupError:
                return False, "the recorder had already exited"
            try:
                recorder.wait(timeout=STOP_WAIT_S)
            except subprocess.TimeoutExpired:
                return False, f"the recorder did not stop within {STOP_WAIT_S:.0f} s"
            return True, "stopped"

    def run_names(self):
        """The run folders' names in the log root (a run folder holds a run.yaml), oldest first. The names start with
        the date and time, so sorting them sorts by age."""
        try:
            names = sorted(os.listdir(self.log_root))
        except OSError:
            return []
        return [name for name in names if os.path.isfile(os.path.join(self.log_root, name, "run.yaml"))]

    def runs(self):
        """The run folders, newest first: [{"name", "size_mb", "closed", "files": [{"path", "size_mb"}]}]. A file's
        path is relative to its run folder, with "/" between folders (bag/bag_0.mcap)."""
        found = []
        for name in reversed(self.run_names()):
            folder = os.path.join(self.log_root, name)
            files = []
            for root, _, file_names in os.walk(folder):
                for file_name in sorted(file_names):
                    full = os.path.join(root, file_name)
                    files.append({"path": os.path.relpath(full, folder).replace(os.sep, "/"),
                                  "size_mb": round(os.path.getsize(full) / 1e6, 2)})
            closed = os.path.isfile(os.path.join(folder, "bag", "metadata.yaml"))
            found.append({"name": name, "size_mb": round(sum(f["size_mb"] for f in files), 2), "closed": closed,
                          "files": sorted(files, key=lambda f: f["path"])})
        return found


def is_own_address(host_header):
    """True if a request's Host header names this machine by IP address, "localhost" or its own
    hostname. A page on another website can be made to look "same-origin" by pointing its own
    domain name at the robot's address; its Host header then carries that foreign name."""
    name = host_header.strip().lower()
    if name.startswith("["):           # an IPv6 address with a port: [::1]:8080
        name = name[1:name.find("]")]
    elif name.count(":") == 1:         # name:port or 192.168.1.42:8080
        name = name.split(":")[0]
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        own = socket.gethostname().lower()
        return name in ("localhost", own, own + ".local")


def make_handler(dashboard, page):
    """The request handler class, bound to this dashboard and the page's bytes."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):  # one log line per request would flood the launch output
            pass

        def send_json(self, body, status=200):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path, _, query = self.path.partition("?")
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif path == "/api/state":
                self.send_json(dashboard.state())
            elif path == "/api/history":
                try:
                    since = float(query.partition("since=")[2] or 0.0)
                except ValueError:
                    since = 0.0
                self.send_json(dashboard.history_since(since))
            elif path == "/api/runs":
                self.send_json(dashboard.runs())
            elif path.startswith("/api/runs/") and path.endswith(".tar"):
                self.send_run(path[len("/api/runs/"):-len(".tar")])
            elif path.startswith("/api/files/"):
                # /api/files/<run name>/<path in the run folder>, both percent-encoded by the page.
                name, _, inner = urllib.parse.unquote(path[len("/api/files/"):]).partition("/")
                self.send_file(name, inner)
            else:
                self.send_json({"error": "not found"}, 404)

        def send_run(self, name):
            # Only a name that runs() itself lists: the browser cannot ask for any other folder.
            run = next((run for run in dashboard.runs() if run["name"] == name), None)
            if run is None:
                self.send_json({"error": "no such run"}, 404)
                return
            if not run["closed"]:
                self.send_json({"error": "this run is still being recorded, or was never closed"}, 409)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/x-tar")
            self.send_header("Content-Disposition", f'attachment; filename="{name}.tar"')
            self.end_headers()  # no Content-Length: the end of the connection ends the file
            # mode "w|" writes the tar straight to the browser, a block at a time. Building it in
            # memory first would hold a whole bag (hundreds of MB) in the Pi's RAM per download.
            with tarfile.open(fileobj=self.wfile, mode="w|") as tar:
                tar.add(os.path.join(dashboard.log_root, name), arcname=name)

        def send_file(self, name, inner):
            """One file of a closed run, e.g. bag/bag_0.mcap for Foxglove. Only a (run, path) pair that runs() lists:
            the path is compared with that list, never joined blindly, so "../" cannot reach outside the run."""
            run = next((run for run in dashboard.runs() if run["name"] == name), None)
            if run is None or inner not in [f["path"] for f in run["files"]]:
                self.send_json({"error": "no such file"}, 404)
                return
            if not run["closed"]:
                self.send_json({"error": "this run is still being recorded, or was never closed"}, 409)
                return
            full = os.path.join(dashboard.log_root, name, *inner.split("/"))
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(os.path.getsize(full)))
            self.send_header("Content-Disposition", f'attachment; filename="{name}_{os.path.basename(full)}"')
            self.end_headers()
            with open(full, "rb") as f:
                shutil.copyfileobj(f, self.wfile)   # a block at a time, never the whole bag in memory

        def do_POST(self):
            # A page on some OTHER website must not be able to press Record through the visitor's
            # browser: a browser sends the Origin of the page that made the request, and it must be
            # this server.
            origin = self.headers.get("Origin")
            host = self.headers.get("Host") or ""
            if (origin is not None and origin.partition("://")[2] != host) or not is_own_address(host):
                self.send_json({"ok": False, "message": "request from another site refused"}, 403)
                return
            try:
                length = int(self.headers.get("Content-Length") or 0)
                # 0 <= : read(-1) would read until the client hangs up, with no size limit.
                body = json.loads(self.rfile.read(length) or b"{}") if 0 <= length <= 4096 else None
            except ValueError:
                body = None
            if not isinstance(body, dict):
                self.send_json({"ok": False, "message": "the request body must be a small JSON object"}, 400)
                return
            if self.path == "/api/record/start":
                ok, message = dashboard.start_recording(
                    str(body.get("label") or "run")[:60], str(body.get("battery_v") or "")[:20],
                    str(body.get("surface") or "")[:60])
            elif self.path == "/api/record/stop":
                ok, message = dashboard.stop_recording()
            elif self.path == "/api/mute":
                ok, message = dashboard.set_muted(bool(body.get("muted")))
                self.send_json({"success": ok, "message": message}, 200 if ok else 504)
                return
            elif self.path in ("/api/enable", "/api/disable", "/api/mode"):
                # The supervisor decides: a refusal comes back as success false with its reason.
                if self.path == "/api/mode":
                    ok, message = dashboard.set_mode(str(body.get("mode") or "")[:30])
                else:
                    ok, message = dashboard.set_enabled(self.path == "/api/enable")
                status = 200 if ok else (504 if message.startswith("supervisor did not answer") else 409)
                self.send_json({"success": ok, "message": message}, status)
                return
            else:
                self.send_json({"error": "not found"}, 404)
                return
            self.send_json({"ok": ok, "message": message}, 200 if ok else 409)

    return Handler


def main():
    try:
        os.nice(NICENESS)  # the control loop comes first
    except OSError:
        pass
    rclpy.init()
    dashboard = Dashboard()
    # The page is installed to share/rocko_env/web/ (CMakeLists.txt).
    with open(os.path.join(get_package_share_directory("rocko_env"), "web", "dashboard.html"), "rb") as f:
        page = f.read()
    try:
        server = ThreadingHTTPServer(("0.0.0.0", dashboard.port), make_handler(dashboard, page))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        # Another dashboard already serves this port: on the Pi, normally the robot launch's own
        # (rocko_robot.launch.py, started by rocko.service), which sees the same topics. Step aside cleanly (exit 0),
        # so a launch file's dashboard:=true does not end in "process has died".
        dashboard.get_logger().info(
            f"dashboard: port {dashboard.port} is already in use by another process (usually the boot "
            "service's dashboard); this one exits and that one keeps serving")
        dashboard.destroy_node()
        rclpy.try_shutdown()
        return
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    dashboard.get_logger().info(
        f"dashboard: open http://<this machine's address>:{dashboard.port} in a browser "
        "(read-only apart from Record/Stop, the supervisor's Enable/Disable/mode buttons and the buzzer's Mute; no login)")
    try:
        rclpy.spin(dashboard)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass  # Ctrl-C, or the launch asked us to stop: both are a normal end
    finally:
        if dashboard.recording_status()["running"]:
            dashboard.stop_recording()  # never leave a bag unclosed behind us
        server.shutdown()


if __name__ == "__main__":
    main()
