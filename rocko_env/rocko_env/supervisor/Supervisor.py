#!/usr/bin/env python3
"""The robot supervisor: owns the robot's state and is the only thing that starts and stops a motor stack.

Design and the rules: docs/architecture/robot-supervisor.md. The rules themselves are in
state_machine.py (no ROS, tested on a laptop); this node does the I/O they ask for:

  * starts / stops a mode's launch file as a child process (own process group, so one signal reaches
    every node of the stack at once);
  * asks controller_manager to activate / deactivate the motion controllers (enable / disable);
  * once a second, reads which controllers and nodes are up, to decide "ready" or "fault";
  * reads /joy for the two gamepad combos and the silent-controller auto-disable;
  * publishes the state on /rocko/state and serves /rocko/set_mode and /rocko/set_enabled.

Nothing here blocks a callback: service calls to controller_manager are asynchronous, and waiting for
the child process is done by polling in a timer. The one blocking wait is at shutdown, after spin ends.
Not on the 100 Hz path: this is a separate process.
"""
import math
import os
import signal
import subprocess
import time

import rclpy
import rclpy.executors
from builtin_interfaces.msg import Time
from controller_manager_msgs.srv import ListControllers, SwitchController
from control_msgs.msg import DynamicJointState
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from rocko_interfaces.msg import ControllerStatus, NodeStatus, RobotState
from rocko_interfaces.srv import SetEnabled, SetMode
from rocko_env.supervisor import modes
from rocko_env.supervisor.state_machine import PHASE_STARTING, StateMachine
from rocko_env.teleop import gamepad

WHEEL_WITH_PITCH = "left_wheel_joint"           # the IMU plugin exports pitch [deg] on both wheel joints
REARM_CONTROLLER = "left_add_feedforward_controller"
REARM_PARAMETER = "rearm_max_pitch_deg"          # [deg] the tipover cutoff's own re-arm band
PITCH_MAX_AGE_S = 1.0                            # [s] older than this, pitch counts as unknown
# [s] older than this, the pitch rate counts as unknown. Short, because the hold-still check (L-8) is about the rate
# right at enable; joint_state_broadcaster publishes it every control cycle (10 ms), so 0.2 s is 20 missed messages.
PITCH_RATE_MAX_AGE_S = 0.2
IGNORE_POLL_AFTER_START_S = 2.0                  # [s] the graph may still list the old stack's nodes
SWITCH_GRACE_S = 5.0                             # [s] after enable/disable, a controller mid-change is not a fault
CALL_GIVE_UP_S = 3.0                             # [s] an unanswered list/get call is dropped and re-asked
TIMER_PERIOD_S = 0.1                             # process / joy-silence checks
POLL_PERIOD_S = 1.0                              # controller_manager and node-graph poll
PUBLISH_PERIOD_S = 0.5                           # 2 Hz
SWITCH_REQUEST_TIMEOUT_S = 5                     # [s, whole seconds: the message field is an int] controller_manager's own limit for one switch
REAP_TIMEOUT_S = 1.0                             # [s] wait after SIGKILL for the dead launch to be collected
ACTIVATE_FAILED_HINT = "restart the stack: choose the mode again"   # the only way out of the fault it causes


class Supervisor(Node):
    def __init__(self):
        super().__init__("supervisor")
        self.declare_parameter("use_mock_hardware", False)
        self.declare_parameter("initial_mode", modes.DEFAULT_MODE)
        self.declare_parameter("stop_timeout_s", 15.0)     # [s] SIGINT, then SIGKILL after this
        self.declare_parameter("start_timeout_s", 60.0)    # [s] to reach "ready"
        self.declare_parameter("log_root", "")             # passed through to the child launch
        # Speed caps per mode, passed through to that mode's launch: <mode>_max_speed, <mode>_max_turn
        # (modes.CAP_ARGS says what they mean). Text, not float: "" (the default) must mean "not set",
        # and the defaults live in the mode launch files, not here.
        for mode in modes.mode_names():
            for name in modes.CAP_ARGS:
                self.declare_parameter(f"{mode}_{name}", "")
        self.use_mock = bool(self.get_parameter("use_mock_hardware").value)
        self.stop_timeout_s = float(self.get_parameter("stop_timeout_s").value)
        self.start_timeout_s = float(self.get_parameter("start_timeout_s").value)
        self.log_root = str(self.get_parameter("log_root").value)
        self.caps = {mode: self.read_caps(mode) for mode in modes.mode_names()}

        self.sm = StateMachine(str(self.get_parameter("initial_mode").value))

        # Child stack
        self.proc = None                 # subprocess.Popen of `ros2 launch`, or None
        self.pgid = None                 # its process group, saved at Popen: getpgid fails once the leader is dead
        self.stopping = False            # a stop has been signalled and is being waited for
        self.stop_deadline_s = 0.0
        self.start_deadline_s = 0.0
        self.started_at_s = 0.0
        self.switch_grace_until_s = 0.0
        # What we last saw
        self.controller_states = {}      # name -> state, from the newest list_controllers ANSWER
        self.controllers_known = False   # True once a stack has answered at least once
        self.cm_answered_s = 0.0         # [s] when controller_manager last answered (or the stack started)
        self.alive_nodes = set()
        self.pitch_deg = None
        self.pitch_time_s = 0.0
        self.pitch_rate_deg_s = None   # [deg/s] + while pitch increases; for request_enable's hold-still check (L-8)
        self.pitch_rate_time_s = 0.0
        self.rearm_max_pitch_deg = None
        # In-flight async calls (one at a time each, so a slow service cannot pile up requests)
        self.list_future = None
        self.param_future = None
        self.list_sent_s = 0.0
        self.list_epoch = 0              # sm.motion_epoch when the in-flight list_controllers was sent
        self.param_sent_s = 0.0
        self.pending_rearm = False
        self.last_logged = None
        self.last_published = None

        qos_state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(RobotState, "/rocko/state", qos_state)
        self.rearm_pub = self.create_publisher(Empty, "/tipover_rearm", 10)
        self.create_service(SetMode, "/rocko/set_mode", self.on_set_mode)
        self.create_service(SetEnabled, "/rocko/set_enabled", self.on_set_enabled)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_joint_states, 10)
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.param_client = self.create_client(GetParameters, f"/{REARM_CONTROLLER}/get_parameters")

        self.create_timer(TIMER_PERIOD_S, self.on_fast_timer)
        self.create_timer(POLL_PERIOD_S, self.poll_stack)
        self.create_timer(PUBLISH_PERIOD_S, self.publish_state)

        self.run_actions()   # the boot action: start the initial mode's stack
        self.get_logger().info(f"supervisor up: mode {self.sm.mode}, disabled, use_mock_hardware={self.use_mock}")

    def read_caps(self, mode):
        """This mode's speed caps. A bad value becomes 0 with an error (modes.checked_caps says why), so
        the stack still starts, and does not drive faster than was asked for."""
        texts = {name: str(self.get_parameter(f"{mode}_{name}").value) for name in modes.CAP_ARGS}
        caps, errors = modes.checked_caps(texts)
        for error in errors:
            self.get_logger().error(f"{mode}_{error}")
        return caps

    # ------------------------------------------------------------------ requests
    def on_set_enabled(self, request, response):
        source = request.source or "dashboard"
        ok, message = self.sm.request_enable(request.enabled, source, *self.balance_inputs())
        response.success, response.message = ok, message
        self.after_change()
        return response

    def on_set_mode(self, request, response):
        source = request.source or "dashboard"
        ok, message = self.sm.request_mode(request.mode, source)
        response.success, response.message = ok, message
        self.after_change()
        return response

    def on_joy(self, msg):
        held = gamepad.pressed_inputs(msg.axes, msg.buttons)
        self.sm.joy_message(held, time.monotonic(), *self.balance_inputs())
        self.after_change()

    def on_joint_states(self, msg):
        for joint, interfaces in zip(msg.joint_names, msg.interface_values):
            if joint != WHEEL_WITH_PITCH:
                continue
            for name, value in zip(interfaces.interface_names, interfaces.values):
                if name == "pitch":
                    self.pitch_deg = value
                    self.pitch_time_s = time.monotonic()
                elif name == "pitch_rate":
                    self.pitch_rate_deg_s = value
                    self.pitch_rate_time_s = time.monotonic()

    def balance_inputs(self):
        """(pitch [deg], re-arm band [deg], pitch rate [deg/s]) for the state machine; None for anything not
        known right now."""
        now = time.monotonic()
        fresh = self.pitch_deg is not None and now - self.pitch_time_s < PITCH_MAX_AGE_S
        rate_fresh = self.pitch_rate_deg_s is not None and now - self.pitch_rate_time_s < PITCH_RATE_MAX_AGE_S
        return ((self.pitch_deg if fresh else None), self.rearm_max_pitch_deg,
                (self.pitch_rate_deg_s if rate_fresh else None))

    # ------------------------------------------------------------------ doing what the rules ask
    def after_change(self):
        """Do what the rules asked, and publish if the state differs from what was last published."""
        self.run_actions()
        self.publish_state(only_if_changed=True)

    def run_actions(self):
        for action in self.sm.take_actions():
            kind = action[0]
            if kind == "START_STACK":
                self.start_stack(action[1])
            elif kind == "STOP_STACK":
                self.stop_stack()
            elif kind == "KILL_STACK":
                self.kill_stack()
            elif kind == "ACTIVATE":
                self.switch_motion(activate=True)
            elif kind == "DEACTIVATE":
                self.switch_motion(activate=False)
            elif kind == "PUBLISH_REARM":
                self.pending_rearm = True    # published once ACTIVATE has succeeded

    def start_stack(self, mode):
        cmd = modes.launch_command(mode, self.use_mock, self.log_root, self.caps[mode])
        self.get_logger().info("starting stack: " + " ".join(cmd))
        # start_new_session: the child gets its own process group, so killpg reaches every node.
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        try:
            self.pgid = os.getpgid(self.proc.pid)
        except ProcessLookupError:
            self.pgid = self.proc.pid   # start_new_session makes the child its own group leader
        self.stopping = False
        self.started_at_s = time.monotonic()
        self.cm_answered_s = self.started_at_s
        self.start_deadline_s = self.started_at_s + self.start_timeout_s
        self.controller_states = {}
        self.controllers_known = False
        self.alive_nodes = set()
        self.rearm_max_pitch_deg = None
        # A call still waiting on the OLD stack's controller_manager will never be answered: forget it.
        self.list_future = None
        self.param_future = None

    def stop_stack(self):
        if self.proc is None or self.proc.poll() is not None:
            # The launch is already gone, but nodes it started may not be: do not leave them holding the motors.
            self.signal_group(signal.SIGKILL)
            self.forget_stack()
            self.sm.stack_stopped()
            return
        self.get_logger().info("stopping stack (SIGINT to its process group)")
        self.signal_group(signal.SIGINT)
        self.stopping = True
        self.stop_deadline_s = time.monotonic() + self.stop_timeout_s

    def signal_group(self, sig):
        # Known, accepted gap: after the group's last process is gone its number could in principle be reused by
        # an unrelated group before this runs. The window is a few milliseconds on a Pi running only ROCKO.
        if self.pgid is None:
            return
        try:
            os.killpg(self.pgid, sig)
        except ProcessLookupError:
            pass   # nothing left in the group

    def forget_stack(self):
        """The child is gone (exited, stopped or killed): nothing of it is left to watch or confirm."""
        self.proc = None
        self.pgid = None
        self.stopping = False
        self.sm.stack_gone()

    def kill_stack(self):
        """SIGKILL the whole group now. A killed motor plugin may leave a PWM pin on (finding H-7); the state
        machine's message says so. Used when controller_manager is silent or a disable cannot be confirmed."""
        self.get_logger().error("killing the stack: nothing else can stop its motors")
        self.signal_group(signal.SIGKILL)
        if self.proc is not None:
            try:
                self.proc.wait(timeout=REAP_TIMEOUT_S)   # SIGKILL cannot be ignored; this reaps the zombie
            except subprocess.TimeoutExpired:
                pass
        self.forget_stack()

    def switch_motion(self, activate):
        names = modes.motion_controllers(self.sm.mode)
        if not self.switch_client.service_is_ready():
            if not activate:
                # Disabled in the state. The state machine keeps asking on every list_controllers
                # answer and kills the stack if none confirms the motion controllers are off.
                self.get_logger().error("cannot deactivate motion controllers yet: controller_manager "
                                        "is not answering; will retry")
            else:
                self.sm.stack_failed(f"cannot enable: controller_manager is not answering; {ACTIVATE_FAILED_HINT}")
            return
        request = SwitchController.Request()
        if activate:
            request.activate_controllers = names
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
        else:
            request.deactivate_controllers = names
            # Best effort on the way down: if one controller is already gone, still stop the rest.
            request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout.sec = SWITCH_REQUEST_TIMEOUT_S
        self.switch_grace_until_s = time.monotonic() + SWITCH_GRACE_S
        future = self.switch_client.call_async(request)
        future.add_done_callback(lambda f, a=activate: self.on_switch_done(f, a))

    def on_switch_done(self, future, activate):
        try:
            ok = future.result().ok
        except Exception as error:   # the service call itself failed
            ok = False
            self.get_logger().error(f"switch_controller call failed: {error}")
        if not ok and activate:
            self.sm.stack_failed(f"controller_manager could not activate the motion controllers; {ACTIVATE_FAILED_HINT}")
            self.pending_rearm = False
        elif not ok:
            # No fault yet: the next list_controllers answer shows whether they are still active, and the
            # state machine retries the deactivate (and kills the stack if it never works).
            self.get_logger().error("controller_manager could not deactivate the motion controllers; will retry")
        elif activate and self.pending_rearm:
            self.rearm_pub.publish(Empty())
        self.pending_rearm = False
        self.after_change()

    # ------------------------------------------------------------------ timers
    def on_fast_timer(self):
        now_s = time.monotonic()
        self.sm.tick(now_s)
        self.check_process(now_s)
        if self.proc is not None and not self.stopping:
            self.sm.controller_manager_unanswered(now_s - self.cm_answered_s)
        self.after_change()

    def check_process(self, now_s):
        if self.stopping:
            if self.proc is None or self.proc.poll() is not None:
                self.signal_group(signal.SIGKILL)   # nodes the launch left behind
                self.forget_stack()
                self.get_logger().info("stack stopped")
                self.sm.stack_stopped()
            elif now_s > self.stop_deadline_s:
                self.kill_stack()
                self.sm.stack_failed(f"stack did not stop in {self.stop_timeout_s:g} s and was killed; "
                                     "check the motors: the motor plugin may have left a wheel's PWM pin switched on")
            return
        if self.proc is not None:
            code = self.proc.poll()
            if code is not None:
                # The launch is gone, but a ros2_control_node it started may still be alive and holding the motors.
                # Accepted in balance mode too: an orphaned motor stack that nothing supervises is worse than the
                # fall. Report the failure FIRST, then forget the stack, so the disable it queues is not left
                # waiting for a confirmation that a dead stack can never give (which would overwrite this reason).
                self.signal_group(signal.SIGKILL)
                self.sm.stack_failed(f"the launch exited on its own (code {code})")
                self.forget_stack()
            elif self.sm.phase == PHASE_STARTING and now_s > self.start_deadline_s:
                self.sm.stack_failed(f"stack not ready after {self.start_timeout_s:g} s")

    def poll_stack(self):
        """Once a second: which controllers and nodes are up? Decides ready, and faults."""
        self.alive_nodes = {name for name, _ns in self.get_node_names_and_namespaces()}
        now_s = time.monotonic()
        if self.list_future is not None and now_s - self.list_sent_s > CALL_GIVE_UP_S:
            # No answer: ask again rather than wait forever. This is NOT "every controller missing": an
            # unanswered question is unknown, and controller_manager_unanswered() (called from the fast
            # timer, counting from cm_answered_s) turns a long silence into a fault. Calling it "missing"
            # here once disabled the whole balance cascade whenever controller_manager was merely slow.
            self.list_future = None
        if self.param_future is not None and now_s - self.param_sent_s > CALL_GIVE_UP_S:
            self.param_future = None
        if self.list_future is None and self.list_client.service_is_ready():
            self.list_sent_s = now_s
            self.list_epoch = self.sm.motion_epoch
            self.list_future = self.list_client.call_async(ListControllers.Request())
            self.list_future.add_done_callback(self.on_list_done)
        # The re-arm band lives in the controller's parameters (one home for the number).
        # Read it as soon as the controller answers, also while starting: asking only once "ready" left a
        # window of up to one poll period in which "enable" was refused with "start pitch unknown".
        if (self.sm.mode == "balance" and self.proc is not None and not self.stopping
                and self.rearm_max_pitch_deg is None and self.param_future is None
                and self.param_client.service_is_ready()):
            request = GetParameters.Request()
            request.names = [REARM_PARAMETER]
            self.param_sent_s = now_s
            self.param_future = self.param_client.call_async(request)
            self.param_future.add_done_callback(self.on_param_done)

    def on_list_done(self, future):
        if future is not self.list_future:
            return   # an answer from a stack we have stopped asking
        self.list_future = None
        try:
            states = {c.name: c.state for c in future.result().controller}
        except Exception as error:
            self.get_logger().warning(f"list_controllers failed: {error}")
            return
        self.controller_states = states
        self.controllers_known = True
        self.cm_answered_s = time.monotonic()
        self.sm.controller_states_reply(states, self.list_epoch)
        self.judge_stack()

    def on_param_done(self, future):
        if future is not self.param_future:
            return
        self.param_future = None
        try:
            value = future.result().values[0].double_value
            if math.isfinite(value) and value > 0.0:
                self.rearm_max_pitch_deg = value
        except Exception as error:
            self.get_logger().warning(f"could not read {REARM_PARAMETER}: {error}")

    def judge_stack(self):
        """Compare what is up with what the mode expects (the rules are StateMachine.observe_stack)."""
        now_s = time.monotonic()
        if (self.proc is None or self.stopping or not self.controllers_known
                or now_s - self.started_at_s < IGNORE_POLL_AFTER_START_S):
            return
        missing_nodes = [n for n in modes.expected_nodes(self.sm.mode)
                         if n not in self.alive_nodes]
        self.sm.observe_stack(self.controller_states, missing_nodes, now_s <= self.switch_grace_until_s)
        self.after_change()

    # ------------------------------------------------------------------ the state, out
    def publish_state(self, only_if_changed=False):
        sm = self.sm
        msg = RobotState()
        now = self.get_clock().now().to_msg()
        msg.stamp = Time(sec=now.sec, nanosec=now.nanosec)
        msg.mode, msg.enabled, msg.phase = sm.mode, sm.enabled, sm.phase
        msg.requested_mode, msg.last_source, msg.message = sm.requested_mode, sm.last_source, sm.message
        msg.available_modes = modes.mode_names()
        for name in modes.expected_nodes(sm.mode):
            msg.nodes.append(NodeStatus(name=name, group=sm.mode, alive=name in self.alive_nodes))
        for name in modes.BASE_NODES:
            msg.nodes.append(NodeStatus(name=name, group="base", alive=name in self.alive_nodes or name == "supervisor"))
        # Not answering is "unknown", not "missing": the last answer is no longer to be believed.
        unanswered = (self.proc is not None and not self.stopping
                      and time.monotonic() - self.cm_answered_s > CALL_GIVE_UP_S)
        for name, expected in sm.expected_controllers():
            state = "unknown" if unanswered else self.controller_states.get(name, "missing")
            msg.controllers.append(ControllerStatus(name=name, state=state, expected_state=expected))
        content = (msg.mode, msg.enabled, msg.phase, msg.requested_mode, msg.last_source, msg.message,
                   tuple((n.name, n.alive) for n in msg.nodes),
                   tuple((c.name, c.state, c.expected_state) for c in msg.controllers))
        if not (only_if_changed and content == self.last_published):
            self.last_published = content
            self.state_pub.publish(msg)

        key = (sm.mode, sm.enabled, sm.phase, sm.requested_mode, sm.message)
        if key != self.last_logged:
            self.last_logged = key
            self.get_logger().info(f"state: mode={sm.mode} enabled={sm.enabled} phase={sm.phase} "
                                   f"source={sm.last_source} message={sm.message!r}")

    # ------------------------------------------------------------------ shutdown
    def shutdown_stack(self):
        """Blocking, and only after spin has ended: stop the child stack the same way as a mode change."""
        if self.proc is None:
            return
        if self.proc.poll() is not None:
            self.signal_group(signal.SIGKILL)   # the launch exited, its nodes may not have
            return
        self.signal_group(signal.SIGINT)
        try:
            self.proc.wait(timeout=self.stop_timeout_s)
        except subprocess.TimeoutExpired:
            self.signal_group(signal.SIGKILL)
            self.proc.wait()
            print(f"supervisor: stack did not stop in {self.stop_timeout_s:g} s and was killed; check the "
                  "motors: the motor plugin may have left a wheel's PWM pin switched on", flush=True)


def main():
    # launch sends SIGINT (rclpy handles it); a plain SIGTERM (systemd, kill) should end spin too.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass   # Ctrl-C, SIGTERM or the launch asking us to stop: a normal end, exit code 0
    except Exception:
        if rclpy.ok():   # an error while ROS was still up is a real one; one during shutdown is not
            raise
    finally:
        # A second SIGINT (the launch and the terminal each send one) must not cut the wait for the stack short.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_stack()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
