#!/usr/bin/env python3
"""Buzzer: plays a short tune when the robot finishes booting, is enabled or disabled, and on a few warnings.

Optional (docs/architecture/robot-extras.md): with no buzzer or no PWM output, it logs the error, reports it on
/diagnostics, tries again every health.RETRY_PERIOD_S, and changes nothing else. Which tune plays when is in
sounds.py; wiring and Pi setup in docs/guides/fit-the-extras.md.

How it makes a note: the buzzer is PASSIVE (a small speaker), so it sounds only while driven with a square wave, at
the wave's frequency. The Pi's hardware PWM makes that wave, set through Linux's /sys/class/pwm files: the period
is 1 / frequency, and the duty cycle is half of it. Hardware PWM keeps the pitch steady however busy the CPU is.

Listens to: /rocko/state (enabled, phase), /joy (is the gamepad there), /rosout (the tipover cutoff's log lines),
/diagnostics (the battery level). Serves: /rocko/set_buzzer_muted (std_srvs/SetBool; the dashboard and the screen
call it). The mute setting is saved to a file, so a muted robot stays quiet after a reboot.
Not on the 100 Hz path: a separate process; the tunes play in their own thread.
"""
import glob
import os
import queue
import threading
import time

import rclpy
import rclpy.executors
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from rocko_interfaces.msg import RobotState
from rocko_env.extras import health, log_phrases, sounds

CHECK_PERIOD_S = 0.1     # [s] how often the state is compared with the last one
GAP_S = 0.03             # [s] silence after each tune, so two tunes in a row are heard as two
MAX_QUEUED = 3           # tunes waiting beyond this are dropped: a sound 5 s late is worse than none


class PwmTone:
    """One hardware PWM channel through /sys/class/pwm. Every call may raise OSError."""

    def __init__(self, chip, channel):
        if chip == "auto":
            chips = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
            if not chips:
                raise OSError("no /sys/class/pwm/pwmchip*: the PWM overlay is not enabled (see the guide)")
            chip = chips[0]
        self.path = os.path.join(chip, f"pwm{channel}")
        if not os.path.isdir(self.path):
            self._write(os.path.join(chip, "export"), channel)
            for _ in range(20):                 # udev may take a moment to make the new files writable: up to 1 s
                if os.access(os.path.join(self.path, "period"), os.W_OK):
                    break
                time.sleep(0.05)
        self.off()

    @staticmethod
    def _write(path, value):
        with open(path, "w") as f:
            f.write(str(value))

    def play(self, frequency_hz):
        if frequency_hz <= 0:
            self.off()
            return
        period_ns = int(1e9 / frequency_hz)
        # The kernel refuses a duty cycle longer than the period, so lower the duty before changing the period.
        self._write(os.path.join(self.path, "duty_cycle"), 0)
        self._write(os.path.join(self.path, "period"), period_ns)
        self._write(os.path.join(self.path, "duty_cycle"), period_ns // 2)   # half on, half off: loudest
        self._write(os.path.join(self.path, "enable"), 1)

    def off(self):
        self._write(os.path.join(self.path, "enable"), 0)


class Buzzer(Node):
    def __init__(self):
        super().__init__("buzzer")
        self.chip = self.declare_parameter("pwm_chip", "auto").value    # "auto": the first /sys/class/pwm/pwmchip*
        self.channel = int(self.declare_parameter("pwm_channel", 0).value)
        self.mute_file = os.path.expanduser(self.declare_parameter("mute_file", "~/.rocko/buzzer_muted").value)

        self.muted = self.load_muted()
        self.events = sounds.SoundEvents()
        self.robot = None                       # (enabled, phase) from /rocko/state
        self.joy_s = None                       # [monotonic s] the last /joy
        self.tipover_by_node = {}               # each wheel's cutoff trips on its own; tripped if any is
        self.battery_level = None
        self.tunes = queue.Queue()
        self.lock = threading.Lock()            # guards self.status, written by the player thread
        self.status = health.report("buzzer", health.ERROR, "starting")
        self.pwm = None                         # the player thread's PwmTone, or None
        self.stopping = threading.Event()       # set by main() at shutdown: the player stops between notes

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(RobotState, "/rocko/state", self.on_state, latched)
        self.create_subscription(Joy, "/joy", self.on_joy, 5)
        self.create_subscription(Log, "/rosout", self.on_log, 20)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.create_service(SetBool, "/rocko/set_buzzer_muted", self.on_set_muted)
        self.health_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_timer(CHECK_PERIOD_S, self.check)
        self.create_timer(health.REPORT_PERIOD_S, self.report)
        self.player_thread = threading.Thread(target=self.player, daemon=True)
        self.player_thread.start()

    # ---- what it listens to ----------------------------------------------------------------------------
    def on_state(self, msg):
        self.robot = (bool(msg.enabled), msg.phase)

    def on_joy(self, _msg):
        self.joy_s = time.monotonic()

    def on_log(self, msg):
        change = log_phrases.tipover_change(msg.msg)
        if change is not None:
            self.tipover_by_node[msg.name] = change

    def on_diagnostics(self, msg):
        battery_report = health.from_message(msg).get("battery")
        if battery_report is not None:
            # No level in an ERROR report (no reading): None, so a lost sensor does not sound like a full battery.
            self.battery_level = battery_report["values"].get("level")

    # ---- deciding --------------------------------------------------------------------------------------
    def check(self):
        enabled, phase = self.robot if self.robot else (None, None)
        age = None if self.joy_s is None else time.monotonic() - self.joy_s
        for name in self.events.update(enabled, phase, any(self.tipover_by_node.values()), age, self.battery_level):
            self.queue_tune(name)

    def queue_tune(self, name):
        if self.muted or self.tunes.qsize() >= MAX_QUEUED:
            return
        self.tunes.put(name)

    def on_set_muted(self, request, response):
        self.muted = bool(request.data)
        try:
            os.makedirs(os.path.dirname(self.mute_file), exist_ok=True)
            with open(self.mute_file, "w") as f:
                f.write("1\n" if self.muted else "0\n")
            saved = ""
        except OSError as error:
            saved = f" (not saved, so a restart forgets it: {error.strerror or error})"
        if not self.muted:
            self.queue_tune("unmuted")
        response.success = True
        response.message = ("muted" if self.muted else "sound on") + saved
        self.get_logger().info(f"buzzer: {response.message}")
        return response

    def load_muted(self):
        try:
            with open(self.mute_file) as f:
                return f.read().strip() == "1"
        except OSError:
            return False   # no file yet: sound on

    def report(self):
        with self.lock:
            status = dict(self.status)
        status["values"] = {**status["values"], "muted": "true" if self.muted else "false"}
        self.health_pub.publish(health.to_message(status))

    # ---- playing, in its own thread so a tune's sleeps never hold up a ROS callback ----------------------
    def set_status(self, level, message):
        with self.lock:
            changed = self.status["message"] != message
            self.status = health.report("buzzer", level, message)
        if changed and level == health.ERROR:
            self.get_logger().error(f"buzzer: {message} (the robot runs without it; retrying every "
                                    f"{health.RETRY_PERIOD_S:.0f} s)")

    def player(self):
        pwm, next_try_s = None, 0.0
        while not self.stopping.is_set():
            if pwm is None and time.monotonic() >= next_try_s:
                try:
                    pwm = PwmTone(self.chip, self.channel)
                    self.pwm = pwm   # for main(), to silence it at shutdown
                    self.set_status(health.OK, f"ready on {pwm.path}")
                except OSError as error:
                    next_try_s = time.monotonic() + health.RETRY_PERIOD_S
                    self.set_status(health.ERROR, f"no PWM output: {error.strerror or error}")
            try:
                name = self.tunes.get(timeout=1.0)
            except queue.Empty:
                continue
            if pwm is None:
                continue   # no buzzer: the tune is dropped, not saved for later
            try:
                for frequency_hz, duration_s in sounds.TUNES[name]:
                    if self.stopping.is_set():
                        break
                    pwm.play(frequency_hz)
                    time.sleep(duration_s)
                pwm.off()
                time.sleep(GAP_S)
            except OSError as error:
                pwm, next_try_s = None, time.monotonic() + health.RETRY_PERIOD_S
                self.pwm = None
                self.set_status(health.ERROR, f"PWM write failed: {error.strerror or error}")


def main():
    rclpy.init()
    node = Buzzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # A stop in the middle of a note would leave the PWM running: a buzzer that whines until the next boot. So stop
        # the player first (it checks between notes; the longest note is 0.4 s), then switch the output off.
        node.stopping.set()
        node.player_thread.join(timeout=1.0)
        if node.pwm is not None:
            try:
                node.pwm.off()
            except OSError:
                pass


if __name__ == "__main__":
    main()
