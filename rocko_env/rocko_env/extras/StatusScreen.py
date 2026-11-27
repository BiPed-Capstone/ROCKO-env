#!/usr/bin/env python3
"""Status screen: shows the robot's state on the 3.5" touch panel on the robot, twice a second.

Optional (docs/architecture/robot-extras.md): with no panel, no touch, or Pillow/numpy missing, it logs the
error, reports it on /diagnostics, tries again every health.RETRY_PERIOD_S, and changes nothing else.
What each page shows is in screen_layout.py (edit that to change the screen); drawing is in screen_draw.py;
wiring and Pi setup in docs/guides/fit-the-extras.md.

How it reaches the panel: the panel's kernel driver (fbtft, enabled by a line in the Pi's config.txt) makes it a
framebuffer, /dev/fbN: a file whose bytes are the pixels. This node draws a picture and writes its bytes there.
Touch: the panel's touch chip (ADS7846) appears as /dev/input/eventN; a tap on the bottom bar's left or right third
changes page, the middle third mutes or unmutes the buzzer (through /rocko/set_buzzer_muted).

preview_png:=/some/file.png draws into that file instead of a panel: how to see the screen on a laptop.
Not on the 100 Hz path: a separate process, at lowered priority.
"""
import fcntl
import os
import queue
import socket
import struct
import threading
import time

import rclpy
import rclpy.executors
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.msg import DynamicJointState
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from rocko_interfaces.msg import RobotState
from rocko_env.extras import health, log_phrases, screen_layout, sysinfo

try:   # optional: without them the robot still runs, and the screen says why it is dark
    from rocko_env.extras import screen_draw
    DRAW_ERROR = None
except ImportError as error:
    screen_draw, DRAW_ERROR = None, f"cannot draw: {error} (install python3-pil and python3-numpy)"

NICENESS = 10            # lower priority than the robot's own programs
GAMEPAD_FRESH_S = 1.0    # [s] /joy newer than this = gamepad connected
PITCH_FRESH_S = 1.0      # [s] pitch older than this is shown as unknown
WHEEL_WITH_PITCH = "left_wheel_joint"   # the IMU plugin exports pitch on the wheel joints

# Linux input events. struct input_event (linux/input.h) is (seconds, microseconds, type, code, value); the numbers
# below are from linux/input-event-codes.h.
EVENT_FORMAT = "llHHi"
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
EV_KEY, EV_ABS = 1, 3            # event types: a button, an absolute axis
ABS_X, ABS_Y = 0, 1
BTN_TOUCH = 330                  # the "finger down" button; value 0 = lifted
ABS_INFO_BYTES = 24              # struct input_absinfo: 6 x int32 (value, minimum, maximum, fuzz, flat, resolution)
IOC_READ = 2                     # an ioctl that reads data back from the driver


def eviocgabs(axis):
    """The ioctl number that asks a touch device for an axis's range (Linux's EVIOCGABS). An ioctl number packs the
    direction, the reply's size, a letter for the driver family ('E' = input) and a command number together."""
    return (IOC_READ << 30) | (ABS_INFO_BYTES << 16) | (ord("E") << 8) | (0x40 + axis)


def find_framebuffer():
    """The panel's /dev/fbN: the framebuffer whose driver name starts with "fb_" (every fbtft driver's does, e.g.
    fb_ili9486), so the HDMI output is never drawn over."""
    names = {}
    for entry in sorted(os.listdir("/sys/class/graphics")) if os.path.isdir("/sys/class/graphics") else []:
        try:
            with open(f"/sys/class/graphics/{entry}/name") as f:
                names[entry] = f.read().strip()
        except OSError:
            continue
    for entry, name in names.items():
        if name.startswith("fb_"):
            return f"/dev/{entry}"
    raise OSError(f"no panel framebuffer (found: {names or 'none'}); is the overlay line in config.txt?")


def find_touch():
    """/dev/input/eventN of the touch chip, from /proc/bus/input/devices."""
    with open("/proc/bus/input/devices") as f:
        blocks = f.read().split("\n\n")
    for block in blocks:
        if "ads7846" in block.lower() or "touchscreen" in block.lower():
            for line in block.splitlines():
                if line.startswith("H: Handlers="):
                    for word in line.split("=", 1)[1].split():
                        if word.startswith("event"):
                            return f"/dev/input/{word}"
    raise OSError("no touch device (ADS7846) in /proc/bus/input/devices")


class Framebuffer:
    def __init__(self, device, rotate_deg):
        base = f"/sys/class/graphics/{os.path.basename(device)}"
        with open(f"{base}/virtual_size") as f:
            self.width, self.height = (int(x) for x in f.read().strip().split(","))
        with open(f"{base}/bits_per_pixel") as f:
            bpp = int(f.read())
        with open(f"{base}/stride") as f:
            self.stride = int(f.read())
        self.rotate_deg = rotate_deg
        size = (screen_layout.WIDTH, screen_layout.HEIGHT)
        turned = size if rotate_deg in (0, 180) else size[::-1]
        if bpp != 16 or (self.width, self.height) != turned:
            raise OSError(f"{device} is {self.width}x{self.height} at {bpp} bits; expected {turned[0]}x{turned[1]} at "
                          f"16 (rotate_deg is {rotate_deg}: try 90 or 270, or the overlay's rotate option)")
        self.file = open(device, "r+b", buffering=0)

    def show(self, image):
        if self.rotate_deg:
            image = image.rotate(self.rotate_deg, expand=True)
        data = screen_draw.to_rgb565(image)
        row = self.width * 2
        self.file.seek(0)
        if self.stride == row:
            self.file.write(data)
        else:   # the driver pads each row: write them one at a time
            for y in range(self.height):
                self.file.seek(y * self.stride)
                self.file.write(data[y * row:(y + 1) * row])


class StatusScreen(Node):
    def __init__(self):
        super().__init__("status_screen")
        self.preview_png = self.declare_parameter("preview_png", "").value
        self.framebuffer_device = self.declare_parameter("framebuffer", "auto").value
        self.touch_device = self.declare_parameter("touch_device", "auto").value
        self.rotate_deg = int(self.declare_parameter("rotate_deg", 0).value)
        # A touch panel's axes need not match the picture's: these fix it (the guide says how to tell).
        self.touch_swap_xy = bool(self.declare_parameter("touch_swap_xy", False).value)
        self.touch_invert_x = bool(self.declare_parameter("touch_invert_x", False).value)
        self.touch_invert_y = bool(self.declare_parameter("touch_invert_y", False).value)
        period_s = float(self.declare_parameter("refresh_period_s", 0.5).value)

        self.page = 0
        self.robot = None
        self.raw_joints = None          # newest /dynamic_joint_states, undecoded (100 Hz: decoding all would waste CPU)
        self.pitch_deg, self.pitch_s = None, None
        self.joy_s = None
        self.tipover_by_node = {}
        self.reports, self.report_s = {}, {}
        self.gains = self.read_gains()
        self.cpu_before = sysinfo.read_cpu_times()
        self.cpu_percent = None

        self.panel = None
        self.next_try_s = 0.0
        self.last_bytes = None
        self.taps = queue.Queue()
        self.touch_problem = None
        self.status = health.report("screen", health.ERROR, "starting")
        self.logged = None

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(RobotState, "/rocko/state", self.on_state, latched)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self.on_joints, newest, raw=True)
        self.create_subscription(Joy, "/joy", self.on_joy, 5)
        self.create_subscription(Log, "/rosout", self.on_log, 20)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.on_diagnostics, 10)
        self.mute_client = self.create_client(SetBool, "/rocko/set_buzzer_muted")
        self.mute_future = None         # the mute request in flight; a tap while one is unanswered is ignored
        self.health_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 5)
        self.create_timer(period_s, self.refresh)
        self.create_timer(health.REPORT_PERIOD_S, self.report)
        if not self.preview_png:
            threading.Thread(target=self.touch_reader, daemon=True).start()

    # ---- what it listens to ----------------------------------------------------------------------------
    def on_state(self, msg):
        self.robot = {"mode": msg.mode, "enabled": bool(msg.enabled), "phase": msg.phase, "message": msg.message,
                      "nodes": [(n.name, bool(n.alive)) for n in msg.nodes],
                      "controllers": [(c.name, c.state, c.expected_state) for c in msg.controllers]}

    def on_joints(self, data):
        self.raw_joints = data

    def on_joy(self, _msg):
        self.joy_s = time.monotonic()

    def on_log(self, msg):
        change = log_phrases.tipover_change(msg.msg)
        if change is not None:
            self.tipover_by_node[msg.name] = change

    def on_diagnostics(self, msg):
        for extra, status in health.from_message(msg).items():
            self.reports[extra], self.report_s[extra] = status, time.monotonic()

    def read_gains(self):
        """(p, i, d) of each balance loop's first wheel, from the installed rocko_controllers.yaml: what balance mode
        loads. A value changed at runtime with `ros2 param set` is not shown."""
        try:
            path = os.path.join(get_package_share_directory("rocko_env"), "config", "rocko_controllers.yaml")
            with open(path) as f:
                config = yaml.safe_load(f)
            gains = {}
            for loop, controller in (("pitch", "pitch_pid_controller"), ("velocity", "velocity_pid_controller")):
                first = next(iter(config[controller]["ros__parameters"]["gains"].values()))
                gains[loop] = (first["p"], first["i"], first["d"])
            return gains
        except (OSError, KeyError, StopIteration, TypeError, yaml.YAMLError) as error:
            self.get_logger().warning(f"status screen: cannot read the gains: {error}")
            return None

    # ---- the view ---------------------------------------------------------------------------------------
    def make_view(self):
        now = time.monotonic()
        if self.raw_joints is not None:
            msg = deserialize_message(self.raw_joints, DynamicJointState)
            self.raw_joints = None
            for joint, interfaces in zip(msg.joint_names, msg.interface_values):
                if joint == WHEEL_WITH_PITCH and "pitch" in interfaces.interface_names:
                    self.pitch_deg = interfaces.values[list(interfaces.interface_names).index("pitch")]
                    self.pitch_s = now
        cpu_now = sysinfo.read_cpu_times()
        if cpu_now and self.cpu_before:
            self.cpu_percent = sysinfo.cpu_percent(self.cpu_before, cpu_now)
        self.cpu_before = cpu_now
        memory = sysinfo.read_memory_mb() or (None, None)

        extras = health.summary(self.reports, {k: now - t for k, t in self.report_s.items()})
        battery_info = extras["battery"]["values"]
        battery_view = None
        if extras["battery"]["state"] in ("OK", "WARN") and "voltage" in battery_info:
            battery_view = {"voltage": float(battery_info["voltage"]), "percent": float(battery_info["percent"]),
                            "level": battery_info["level"]}
        muted_text = extras["buzzer"]["values"].get("muted")
        try:
            ips = sysinfo.ipv4_addresses()
        except OSError:
            ips = {}
        return {
            "robot": self.robot,
            "pitch_deg": self.pitch_deg if self.pitch_s and now - self.pitch_s < PITCH_FRESH_S else None,
            "battery": battery_view,
            "gamepad": self.joy_s is not None and now - self.joy_s < GAMEPAD_FRESH_S,
            "tipover": any(self.tipover_by_node.values()) if self.tipover_by_node else None,
            "gains": self.gains,
            "ips": ips,
            "system": {"cpu_percent": self.cpu_percent, "ram_used_mb": memory[0], "ram_total_mb": memory[1],
                       "temp_c": sysinfo.read_temperature_c(), "uptime_s": sysinfo.read_uptime_s(),
                       "hostname": socket.gethostname()},
            "extras": extras,
            "muted": None if muted_text is None else muted_text == "true",
        }

    # ---- drawing ----------------------------------------------------------------------------------------
    def fail(self, message):
        self.panel = None
        self.last_bytes = None
        self.next_try_s = time.monotonic() + health.RETRY_PERIOD_S
        self.status = health.report("screen", health.ERROR, message)
        if message != self.logged:
            self.logged = message
            self.get_logger().error(f"status screen: {message} (the robot runs without it; retrying every "
                                    f"{health.RETRY_PERIOD_S:.0f} s)")

    def refresh(self):
        while not self.taps.empty():
            self.on_tap(self.taps.get())
        if screen_draw is None:
            if self.status["message"] != DRAW_ERROR:
                self.fail(DRAW_ERROR)
            return
        view = self.make_view()
        image = screen_draw.render(view, self.page)
        if self.preview_png:
            image.save(self.preview_png)
            self.status = health.report("screen", health.OK, f"preview to {self.preview_png}")
            return
        if self.panel is None:
            if time.monotonic() < self.next_try_s:
                return
            try:
                device = find_framebuffer() if self.framebuffer_device == "auto" else self.framebuffer_device
                self.panel = Framebuffer(device, self.rotate_deg)
                self.get_logger().info(f"status screen: drawing on {device}")
            except (OSError, ValueError) as error:
                self.fail(str(error))
                return
        data = image.tobytes()
        if data == self.last_bytes:
            return   # nothing changed: the panel's SPI link is slow, so do not resend the same picture
        try:
            self.panel.show(image)
        except OSError as error:
            self.fail(f"writing to the panel failed: {error.strerror or error}")
            return
        self.last_bytes = data
        if self.touch_problem:
            self.status = health.report("screen", health.WARN, f"drawing, but touch: {self.touch_problem}")
        else:
            self.status = health.report("screen", health.OK, f"page {screen_layout.PAGES[self.page][0]}")
        self.logged = None

    def report(self):
        values = {**self.status["values"], "page": screen_layout.PAGES[self.page][0]}
        self.health_pub.publish(health.to_message({**self.status, "values": values}))

    # ---- touch --------------------------------------------------------------------------------------------
    def on_tap(self, button):
        if button == "prev":
            self.page = (self.page - 1) % len(screen_layout.PAGES)
        elif button == "next":
            self.page = (self.page + 1) % len(screen_layout.PAGES)
        elif button == "mute":
            muted = self.reports.get("buzzer", {}).get("values", {}).get("muted")
            if muted is None or not self.mute_client.service_is_ready():
                self.get_logger().warning("status screen: mute tapped, but the buzzer is not running")
                return
            if self.mute_future is not None and not self.mute_future.done():
                return   # the last tap is not answered yet: do not pile up requests behind a stuck buzzer
            # The answer is not needed: the buzzer's next report shows the new setting on the button.
            self.mute_future = self.mute_client.call_async(SetBool.Request(data=muted != "true"))
        self.last_bytes = None   # redraw now

    def touch_reader(self):
        """Its own thread: blocks reading touch events, and queues a button for each finished tap."""
        while rclpy.ok():
            try:
                device = find_touch() if self.touch_device == "auto" else self.touch_device
                with open(device, "rb", buffering=0) as f:
                    ranges = {}
                    for axis in (ABS_X, ABS_Y):
                        info = fcntl.ioctl(f, eviocgabs(axis), bytes(ABS_INFO_BYTES))
                        _, low, high = struct.unpack("6i", info)[:3]
                        ranges[axis] = (low, max(high, low + 1))
                    self.touch_problem = None
                    raw = {ABS_X: 0, ABS_Y: 0}
                    while rclpy.ok():
                        _, _, kind, code, value = struct.unpack(EVENT_FORMAT, f.read(EVENT_SIZE))
                        if kind == EV_ABS and code in raw:
                            raw[code] = value
                        elif kind == EV_KEY and code == BTN_TOUCH and value == 0:   # finger lifted: a tap
                            button = self.button_for(raw, ranges)
                            if button is not None:   # a tap above the bottom bar does nothing
                                self.taps.put(button)
            except (OSError, struct.error) as error:
                self.touch_problem = str(error)
                time.sleep(health.RETRY_PERIOD_S)

    def button_for(self, raw, ranges):
        """Raw touch readings -> screen pixel -> button (screen_layout.button_at)."""
        fx = (raw[ABS_X] - ranges[ABS_X][0]) / (ranges[ABS_X][1] - ranges[ABS_X][0])
        fy = (raw[ABS_Y] - ranges[ABS_Y][0]) / (ranges[ABS_Y][1] - ranges[ABS_Y][0])
        if self.touch_swap_xy:
            fx, fy = fy, fx
        if self.touch_invert_x:
            fx = 1.0 - fx
        if self.touch_invert_y:
            fy = 1.0 - fy
        return screen_layout.button_at(fx * screen_layout.WIDTH, fy * screen_layout.HEIGHT)


def main():
    try:
        os.nice(NICENESS)
    except OSError:
        pass
    rclpy.init()
    node = StatusScreen()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
