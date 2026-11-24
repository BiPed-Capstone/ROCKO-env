#!/usr/bin/env python3
"""The checks of verify_supervisor.sh (Tier 2, mock hardware): drive the robot supervisor through its
rules and print one PASS/FAIL line per check. Run by that script, inside the container, with
`ros2 launch rocko_env rocko_robot.launch.py use_mock_hardware:=true ...` already up. Exit 0 = all passed.

Everything is waited on by polling /rocko/state (or /joint_states) with a bounded timeout, never by a
fixed sleep for an event this script does not control. The few fixed delays are the ones this script
itself controls: how long a simulated button is held down, and how long it is then released.

What it cannot show: the mock echoes each wheel command back as the wheel's velocity and holds pitch at
0.0, so this is a check of wiring and rules, not of balance. See docs/architecture/robot-supervisor.md.
"""
import json
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState

from rocko_interfaces.msg import RobotState
from rocko_interfaces.srv import SetEnabled, SetMode

BOOT_TIMEOUT_S = 60.0       # [s] the supervisor's own start_timeout_s default
SWITCH_TIMEOUT_S = 60.0     # [s] a mode change (stop one stack, start the other)
ACTION_TIMEOUT_S = 10.0     # [s] enable / disable / combo / one controller change
FAULT_TIMEOUT_S = 10.0      # [s] a killed controller_manager must show as a fault within this
NODE_LOSS_TIMEOUT_S = 40.0  # [s] a SIGKILLed node stays in the ROS graph until DDS gives up on it
AUTO_DISABLE_LIMIT_S = 1.0  # [s] silent controller -> disabled (rule 5: 0.25 s timeout + 0.1 s timer)
STOP_LIMIT_S = 1.0          # [s] disable -> wheel velocity reads zero
JOY_RATE_HZ = 20.0
HOLD_S = 0.4                # [s] a simulated combo is held this long, then released this long
MOVING_RAD_S = 0.5          # [rad/s] a wheel faster than this counts as moving (a full stick is far above)
STILL_RAD_S = 1e-3          # [rad/s] a wheel slower than this counts as stopped

TANK_MOTION = ["diffbot_base_controller"]
CASCADE = ["left_add_feedforward_controller", "right_add_feedforward_controller",
           "pitch_pid_controller", "velocity_pid_controller"]


class Abort(Exception):
    """A prerequisite failed, so the later checks cannot say anything."""


class Flow(Node):
    def __init__(self):
        super().__init__("supervisor_flow_check")
        self.lock = threading.Lock()
        self.state = None            # newest /rocko/state
        self.history = []            # (monotonic s, RobotState), every message heard
        self.wheel_speed = None      # [rad/s] newest max |wheel velocity| from /joint_states
        self.wheel_time = 0.0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotState, "/rocko/state", self.on_state, qos)
        self.joints_sub = None       # /joint_states (100 Hz) is heard only while a check needs it, see watch_wheels
        self.set_enabled_client = self.create_client(SetEnabled, "/rocko/set_enabled")
        self.set_mode_client = self.create_client(SetMode, "/rocko/set_mode")
        # The /joy stream runs in a PROCESS of its own (joy_publisher_main): a thread in this process was
        # starved for up to 0.5 s now and then by the executor's callbacks, and a pause past the supervisor's
        # 0.25 s silence limit disabled the robot in the middle of a check.
        self._joy_on = True
        self._joy_frame = {}
        self.joy_process = subprocess.Popen([sys.executable, __file__, "--joy-publisher"], stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, text=True)
        self.results = []

    # ---- inputs -------------------------------------------------------------------------------
    def on_state(self, msg):
        with self.lock:
            self.state = msg
            self.history.append((time.monotonic(), msg))

    def on_joints(self, msg):
        speeds = [abs(v) for n, v in zip(msg.name, msg.velocity) if n.endswith("wheel_joint")]
        if speeds:
            with self.lock:
                self.wheel_speed, self.wheel_time = max(speeds), time.monotonic()

    def watch_wheels(self, on):
        """Listen to /joint_states only while a check needs the wheel speed: 100 Hz of Python callbacks
        beside the /joy thread stalled that stream past the supervisor's 0.25 s silence limit."""
        if on and self.joints_sub is None:
            self.joints_sub = self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        elif not on and self.joints_sub is not None:
            self.destroy_subscription(self.joints_sub)
            self.joints_sub = None

    @property
    def joy_on(self):
        return self._joy_on

    @joy_on.setter
    def joy_on(self, value):
        self._joy_on = bool(value)
        self.send_joy_settings()

    @property
    def joy_frame(self):
        return self._joy_frame

    @joy_frame.setter
    def joy_frame(self, frame):
        self._joy_frame = frame
        self.send_joy_settings()

    def send_joy_settings(self):
        self.joy_process.stdin.write(json.dumps({"on": self._joy_on, "frame": self._joy_frame}) + "\n")
        self.joy_process.stdin.flush()

    def stop_joy(self):
        """End the /joy process; returns the longest pause (s) between two of its sends while on."""
        self.joy_process.stdin.close()
        try:
            return float(self.joy_process.stdout.readline() or "nan")
        finally:
            self.joy_process.wait(timeout=10)

    @staticmethod
    def make_joy(lb=False, rb=False, lt=False, rt=False, left_y=0.0, right_x=0.0):
        """A frame for the /joy process: the buttons and sticks (build_joy turns it into a message)."""
        return {"lb": lb, "rb": rb, "lt": lt, "rt": rt, "left_y": left_y, "right_x": right_x}

    def press(self, **held):
        """Hold a combo for HOLD_S, then show everything released for HOLD_S (so the next press is a new
        rising edge). The joy stream keeps its 20 Hz rate throughout."""
        self.joy_frame = self.make_joy(**held)
        time.sleep(HOLD_S)
        self.joy_frame = self.make_joy()
        time.sleep(HOLD_S)

    # ---- reading the state ----------------------------------------------------------------------
    def now_state(self):
        with self.lock:
            return self.state

    def controller_states(self):
        s = self.now_state()
        return {c.name: c.state for c in s.controllers} if s else {}

    def describe(self):
        s = self.now_state()
        if s is None:
            return "no /rocko/state heard"
        return (f"mode={s.mode} enabled={s.enabled} phase={s.phase} source={s.last_source} "
                f"message={s.message!r} controllers={ {c.name: c.state for c in s.controllers} }")

    def wait_until(self, predicate, timeout_s, period_s=0.05):
        """Poll until predicate() is true. Returns (ok, seconds waited)."""
        start = time.monotonic()
        while True:
            if predicate():
                return True, time.monotonic() - start
            if time.monotonic() - start >= timeout_s:
                return False, time.monotonic() - start
            time.sleep(period_s)

    def ready_in(self, mode, timeout_s):
        def pred():
            s = self.now_state()
            return s is not None and s.mode == mode and s.phase == "ready" and not s.enabled
        return self.wait_until(pred, timeout_s)

    def controllers_in(self, names, state, timeout_s):
        return self.wait_until(
            lambda: all(self.controller_states().get(n) == state for n in names), timeout_s)

    # ---- services -------------------------------------------------------------------------------
    def call(self, client, request, timeout_s=10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        future = client.call_async(request)
        ok, _ = self.wait_until(future.done, timeout_s)
        return future.result() if ok else None

    def set_enabled(self, enabled):
        request = SetEnabled.Request()
        request.enabled, request.source = enabled, "dashboard"
        return self.call(self.set_enabled_client, request)

    def set_enabled_balance(self):
        """Enable in balance mode. The supervisor reads the re-arm band from the controller once a second,
        so for up to a second after "ready" it refuses with "unknown": that refusal is asked again."""
        start = time.monotonic()
        while True:
            response = self.set_enabled(True)
            if (response is None or response.success or "unknown" not in response.message
                    or time.monotonic() - start > ACTION_TIMEOUT_S):
                return response
            time.sleep(0.2)

    def set_mode(self, mode):
        request = SetMode.Request()
        request.mode, request.source = mode, "dashboard"
        return self.call(self.set_mode_client, request)

    # ---- reporting ------------------------------------------------------------------------------
    def check(self, name, ok, detail=""):
        self.results.append(bool(ok))
        print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""), flush=True)
        return ok

    def require(self, name, ok, detail=""):
        """A check the later ones depend on: a failure ends the run."""
        if not self.check(name, ok, detail):
            print(f"  state: {self.describe()}", flush=True)
            raise Abort(name)


def kill_controller_manager():
    """SIGKILL the child stack's ros2_control_node (the pattern needs the leading slash: it is a path)."""
    return subprocess.run(["pkill", "-KILL", "-f", "/ros2_control_node"]).returncode == 0


def run_checks(f):
    # ---- 1. boot ----------------------------------------------------------------------------
    ok, took = f.ready_in("tank", BOOT_TIMEOUT_S)
    f.require("1  boots: mode tank, disabled, phase ready", ok, f"{took:.1f} s after the checker started")
    states = f.controller_states()
    f.check("1  diffbot_base_controller inactive, joint_state_broadcaster active",
            states.get("diffbot_base_controller") == "inactive"
            and states.get("joint_state_broadcaster") == "active", str(states))

    # ---- 2. enable ----------------------------------------------------------------------------
    response = f.set_enabled(True)
    f.require("2  set_enabled true -> success", response is not None and response.success,
              response.message if response else "no answer")
    ok, took = f.controllers_in(TANK_MOTION, "active", ACTION_TIMEOUT_S)
    f.require("2  diffbot_base_controller active", ok, f"{took:.2f} s")

    # ---- 3. drive. tank_teleop publishes zero unless enabled + /joy heard + sticks seen centred since
    # enable; the joy stream has been showing centred sticks all along, so pushing the stick now drives.
    f.watch_wheels(True)
    f.joy_frame = f.make_joy(left_y=1.0)
    ok, took = f.wait_until(lambda: (f.wheel_speed or 0.0) > MOVING_RAD_S, ACTION_TIMEOUT_S)
    f.check("3  enabled + stick forward -> wheels move (mock echoes the command)", ok,
            f"|wheel velocity| {f.wheel_speed} rad/s after {took:.2f} s")

    # ---- 4. mode change refused while enabled --------------------------------------------------
    response = f.set_mode("balance")
    s = f.now_state()
    f.check("4  set_mode balance while enabled -> refused, mode stays tank",
            response is not None and not response.success and s.mode == "tank" and s.enabled
            and s.phase == "ready", f"{response.message if response else 'no answer'}; {f.describe()}")

    # ---- 3b. disable stops the wheels (stick is still pushed forward) -------------------------
    response = f.set_enabled(False)
    t_disable = time.monotonic()
    ok, _ = f.wait_until(lambda: f.wheel_time > t_disable and (f.wheel_speed or 0.0) < STILL_RAD_S,
                         STOP_LIMIT_S)
    f.check(f"3  disable (stick still forward) -> wheels read 0 within {STOP_LIMIT_S:g} s",
            response is not None and response.success and ok,
            f"{time.monotonic() - t_disable:.2f} s, |wheel velocity| {f.wheel_speed} rad/s")
    f.joy_frame = f.make_joy()
    f.watch_wheels(False)
    ok, _ = f.controllers_in(TANK_MOTION, "inactive", ACTION_TIMEOUT_S)
    f.check("3  diffbot_base_controller inactive again", ok)

    # ---- 5. tank -> balance, disabled --------------------------------------------------------
    t_request = time.monotonic()
    n_before = len(f.history)
    response = f.set_mode("balance")
    f.require("5  set_mode balance while disabled -> accepted",
              response is not None and response.success, response.message if response else "no answer")
    ok, took = f.ready_in("balance", SWITCH_TIMEOUT_S)
    switch_s = time.monotonic() - t_request
    with f.lock:
        phases = [m.phase for _, m in f.history[n_before:]]
    f.require("5  phase left ready and returned to ready with mode balance", ok and any(p != "ready" for p in phases), f"phases seen: {phases}")
    f.check("5  the four cascade controllers inactive",
            all(f.controller_states().get(n) == "inactive" for n in CASCADE), str(f.controller_states()))
    print(f"INFO  switch tank -> balance (request to ready): {switch_s:.1f} s", flush=True)

    # ---- 6. enable in balance ---------------------------------------------------------------
    response = f.set_enabled_balance()
    f.require("6  set_enabled true in balance (mock pitch 0) -> success",
              response is not None and response.success, response.message if response else "no answer")
    ok, took = f.controllers_in(CASCADE, "active", ACTION_TIMEOUT_S)
    f.check("6  cascade controllers active", ok, f"{took:.2f} s; {f.controller_states()}")
    response = f.set_enabled(False)
    ok, _ = f.controllers_in(CASCADE, "inactive", ACTION_TIMEOUT_S)
    f.check("6  disable -> cascade controllers inactive", response is not None and response.success and ok)

    # ---- 7. combos from /joy ------------------------------------------------------------------
    # The joy stream has been showing everything released, so the combo detector is armed.
    f.press(lb=True, rb=True)
    ok, took = f.wait_until(lambda: f.now_state().enabled, ACTION_TIMEOUT_S)
    f.check("7  LB+RB -> enabled (balance)", ok and f.now_state().last_source == "gamepad", f.describe())
    f.press(lb=True, rb=True)
    ok, took = f.wait_until(lambda: not f.now_state().enabled, ACTION_TIMEOUT_S)
    f.check("7  LB+RB again -> disabled", ok and f.now_state().last_source == "gamepad", f.describe())
    t_request = time.monotonic()
    f.press(lt=True, rt=True, rb=True)
    ok, took = f.ready_in("tank", SWITCH_TIMEOUT_S)
    switch_s = time.monotonic() - t_request   # from the moment the combo went down
    f.check("7  LT+RT+RB -> mode back to tank, ready", ok, f.describe())
    print(f"INFO  switch balance -> tank by combo (combo down to ready, includes the 0.4 s release): {switch_s:.1f} s", flush=True)
    f.press(lb=True, rb=True)
    ok, _ = f.wait_until(lambda: f.now_state().enabled, ACTION_TIMEOUT_S)
    f.require("7  LB+RB -> enabled (tank)", ok, f.describe())
    f.press(lt=True, rt=True, rb=True)
    s = f.now_state()
    f.check("7  LT+RT+RB while enabled -> refused (still tank, enabled, ready)",
            s.mode == "tank" and s.enabled and s.phase == "ready" and "disable" in s.message, f.describe())
    f.press(lb=True, rb=True)
    ok, _ = f.wait_until(lambda: not f.now_state().enabled, ACTION_TIMEOUT_S)
    f.require("7  LB+RB -> disabled (tank)", ok, f.describe())

    # ---- 8. auto-disable when /joy goes silent --------------------------------------------------
    f.press(lb=True, rb=True)
    ok, _ = f.wait_until(lambda: f.now_state().enabled, ACTION_TIMEOUT_S)
    f.require("8  enabled by LB+RB with /joy streaming", ok, f.describe())
    f.joy_on = False
    t_silent = time.monotonic()
    ok, took = f.wait_until(lambda: not f.now_state().enabled, 5.0, period_s=0.01)
    took = time.monotonic() - t_silent
    s = f.now_state()
    f.check(f"8  /joy silent -> disabled within {AUTO_DISABLE_LIMIT_S:g} s, last_source supervisor",
            ok and took <= AUTO_DISABLE_LIMIT_S and s.last_source == "supervisor",
            f"{took:.2f} s; source={s.last_source!r} message={s.message!r}")
    f.joy_frame = f.make_joy()
    f.joy_on = True

    # ---- 9. fault: kill controller_manager while enabled -----------------------------------------
    # No /joy at all while enabled, so the silent-controller rule (8) cannot be what disables the robot:
    # only the supervisor noticing that controller_manager stopped answering can.
    f.joy_on = False
    f.joy_frame = f.make_joy()
    f.require("9  (setup) tank ready, enabled with no /joy streaming",
              f.ready_in("tank", ACTION_TIMEOUT_S)[0] and f.set_enabled(True).success
              and f.controllers_in(TANK_MOTION, "active", ACTION_TIMEOUT_S)[0], f.describe())
    f.require("9  (setup) killed ros2_control_node", kill_controller_manager(), "pkill -KILL on ros2_control_node")
    t_kill = time.monotonic()
    ok, took = f.wait_until(lambda: f.now_state().phase == "fault", FAULT_TIMEOUT_S)
    s = f.now_state()
    f.check(f"9  controller_manager killed -> phase fault, disabled, message about controller_manager, "
            f"within {FAULT_TIMEOUT_S:g} s",
            ok and not s.enabled and "controller_manager" in s.message and s.last_source == "supervisor",
            f"{time.monotonic() - t_kill:.1f} s; {f.describe()}")
    phase_before_change = f.now_state().phase
    response = f.set_mode("tank")
    f.require("9  set_mode tank accepted FROM fault",
              phase_before_change == "fault" and response is not None and response.success,
              f"phase was {phase_before_change!r}; {response.message if response else 'no answer'}")
    ok, took = f.ready_in("tank", SWITCH_TIMEOUT_S)
    states = f.controller_states()
    f.check("9  recovers: phase ready, mode tank, every controller in its expected state",
            ok and states == {"joint_state_broadcaster": "active", "diffbot_base_controller": "inactive"},
            f"{time.monotonic() - t_kill:.1f} s after the kill; {f.describe()}")
    f.joy_on = True

    # ---- 11. balance: a lost helper node is a fault but the robot STAYS enabled -----------------
    # (docs/architecture/robot-supervisor.md rule 5: disabling balance drops the robot.)
    response = f.set_mode("balance")
    f.require("11 (setup) set_mode balance accepted",
              response is not None and response.success, response.message if response else "no answer")
    f.require("11 (setup) balance ready", f.ready_in("balance", SWITCH_TIMEOUT_S)[0], f.describe())
    response = f.set_enabled_balance()
    f.require("11 (setup) enabled in balance", response is not None and response.success
              and f.controllers_in(CASCADE, "active", ACTION_TIMEOUT_S)[0],
              response.message if response else "no answer")
    f.require("11 (setup) killed BalanceTeleop.py", subprocess.run(["pkill", "-KILL", "-f", "BalanceTeleop.py"]).returncode == 0)
    ok, took = f.wait_until(lambda: f.now_state().phase == "fault", NODE_LOSS_TIMEOUT_S)
    s = f.now_state()
    f.check("11 balance_teleop killed -> phase fault, STILL enabled, cascade still active",
            ok and s.enabled and "balance_teleop" in s.message
            and all(f.controller_states().get(n) == "active" for n in CASCADE),
            f"{took:.1f} s; {f.describe()}")
    response = f.set_enabled(False)
    ok, _ = f.controllers_in(CASCADE, "inactive", ACTION_TIMEOUT_S)
    f.check("11 disable by service -> cascade inactive", response is not None and response.success and ok,
            f.describe())
    response = f.set_mode("tank")
    ok, _ = f.ready_in("tank", SWITCH_TIMEOUT_S)
    f.check("11 set_mode tank from fault -> ready", response is not None and response.success and ok, f.describe())


def build_joy(frame):
    """A /joy message in joy's game_controller layout (teleop/gamepad.py): 6 axes, 21 buttons.
    A pressed trigger reads -1.0 (the joy node's sign); sticks are 0.0 when centred."""
    msg = Joy()
    msg.axes = [0.0, float(frame.get("left_y", 0.0)), float(frame.get("right_x", 0.0)), 0.0,
                -1.0 if frame.get("lt") else 0.0, -1.0 if frame.get("rt") else 0.0]
    msg.buttons = [0] * 21
    msg.buttons[9], msg.buttons[10] = int(bool(frame.get("lb"))), int(bool(frame.get("rb")))
    return msg


def joy_publisher_main():
    """The /joy process: publishes the frame it was last told about at JOY_RATE_HZ, until its stdin closes,
    then prints the longest pause between two sends. Each stdin line is {"on": bool, "frame": {...}}."""
    rclpy.init()
    node = rclpy.create_node("supervisor_flow_joy")
    publisher = node.create_publisher(Joy, "/joy", 10)
    settings = {"on": True, "frame": {}}
    done = threading.Event()

    def read_commands():
        for line in sys.stdin:
            settings.update(json.loads(line))
        done.set()

    threading.Thread(target=read_commands, daemon=True).start()
    next_s = time.monotonic()
    last_send_s, longest_gap_s = None, 0.0
    while not done.is_set():
        now_s = time.monotonic()
        if settings["on"]:
            if last_send_s is not None:
                longest_gap_s = max(longest_gap_s, now_s - last_send_s)
            last_send_s = now_s
            publisher.publish(build_joy(settings["frame"]))
        else:
            last_send_s = None   # a deliberate silence is not a stall
        next_s += 1.0 / JOY_RATE_HZ
        time.sleep(max(0.0, next_s - time.monotonic()))
    print(f"{longest_gap_s:.3f}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


def main():
    if "--joy-publisher" in sys.argv:
        joy_publisher_main()
        return 0
    rclpy.init()
    flow = Flow()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(flow)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        run_checks(flow)
    except Abort as stop:
        print(f"FAIL  run stopped early: '{stop}' failed, so the later checks did not run", flush=True)
        flow.results.append(False)
    except Exception as error:   # a bug in this script is a failure, not a pass
        print(f"FAIL  checker crashed: {type(error).__name__}: {error}", flush=True)
        flow.results.append(False)
    joy_gap_max_s = flow.stop_joy()
    passed = sum(flow.results)
    # If a combo check fails with "silent", this says whether this script's own /joy stream stalled
    # (the real joy node republishes every 0.05 s; the supervisor disables after 0.25 s of silence).
    print(f"INFO  longest pause in this script's /joy stream: {joy_gap_max_s:.3f} s", flush=True)
    print(f"checks: {passed} passed, {len(flow.results) - passed} failed", flush=True)
    executor.shutdown()
    rclpy.try_shutdown()
    return 0 if flow.results and all(flow.results) else 1


if __name__ == "__main__":
    sys.exit(main())
