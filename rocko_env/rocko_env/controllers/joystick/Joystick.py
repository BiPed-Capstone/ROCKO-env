#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
import time
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, Twist, Vector3 # not sure if needed
import math

def expo_curve(value: float, expo: float = 0.45) -> float:
    return (1 - expo) * value + expo * math.pow(value, 3)

class Joystick(Node):

    # Gamepad message layout (unchanged): 6 float32 axes then 16 uint8 buttons.
    NUM_AXES = 6
    NUM_BUTTONS = 16
    # pygame reports each axis in [-1, 1]. Anything outside that by more than this (float32
    # rounding needs far less) means the bytes are not a gamepad message: a different sender, or
    # the stream lost its place between messages.
    AXIS_TOLERANCE = 0.01
    # A NEW TCP connection replaces the current client only if the current one has sent nothing for
    # longer than this [s]. A link that died silently never reports itself, so a replacement must be
    # possible; but a live laptop sends a heartbeat every 0.05 s (computer_joystick.py
    # HEARTBEAT_PERIOD_S), so a stranger who connects while the real gamepad is talking is refused
    # and cannot take over the robot's commands. 0.5 s is ten heartbeats.
    CLIENT_SILENT_REPLACE_S = 0.5
    # Most bytes read from the client in one 10 ms tick [bytes]. A real gamepad sends 40 bytes per
    # message at most a few hundred times a second; a flood must not keep this node (and so the
    # executor it shares) busy forever. Under a flood the backlog waits in the kernel's buffer and
    # the commands get older; DiffDriveController's watchdog only sees that commands still ARRIVE.
    MAX_BYTES_PER_TICK = 4096

    def __init__(self):
        super().__init__('joystick')

        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)


        self.server_sock.bind(("0.0.0.0", 1234))
        self.server_sock.listen(1)
        # Non-blocking: the timer below accepts the gamepad, so a restarted computer_joystick.py
        # (or one whose link died silently) can connect later. A blocking accept() here would
        # be the only chance to connect, and the node could not run until then.
        self.server_sock.setblocking(False)
        self.client_sock = None
        self.rx_buffer = b""
        self.now = time.monotonic        # [s], a monotonic clock; a test replaces it
        self.last_rx_s = self.now()      # [s] when the current client last sent bytes (or connected)
        self.message_size = struct.calcsize(f"{self.NUM_AXES}f {self.NUM_BUTTONS}B")
        self.get_logger().info("Waiting for joystick connection...")

        # Set up publisher 
        self.joystick_topic = self.create_publisher(Joy, 'joystick', 10)
        self.robot_body_vector_updated_topic = self.create_publisher(TwistStamped, 'diffbot_base_controller/cmd_vel', 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.receive_joystick_data)

        self.linear_scale = -10.0  
        self.angular_scale = -35.0  
        self.deadband = 0.05 


    def receive_joystick_data(self):
        self.accept_new_client()
        message = self.read_newest_message()
        if message is None:
            # Nothing new arrived. Deliberately publish NOTHING: this node used to
            # re-publish the last command every 10 ms, which made an old command look
            # fresh. DiffDriveController holds the last command and ages it out
            # (command_timeout_s), so it must only ever see commands that really arrived.
            return
        self.convert_to_twist(message)
        self.joystick_topic.publish(message)

    def accept_new_client(self):
        try:
            new_sock, _ = self.server_sock.accept()
        except BlockingIOError:
            return  # nobody is connecting
        except OSError as error:
            # e.g. ECONNABORTED: the peer gave up between the handshake and accept(). That is the
            # peer's problem; the node (and the current client, if any) carries on.
            self.get_logger().warning(f"accept() failed ({error}); ignoring this connection attempt.")
            return
        if self.client_sock is not None:
            silent_s = self.now() - self.last_rx_s
            if silent_s <= self.CLIENT_SILENT_REPLACE_S:
                # The current client is alive (it heartbeats every 0.05 s): do not let a newcomer
                # take its place. Throttled, so a stranger who keeps retrying cannot flood the log.
                new_sock.close()
                self.get_logger().warning(
                    "Refused a second joystick connection: the current one sent data "
                    f"{silent_s:.2f} s ago (it is replaced only after {self.CLIENT_SILENT_REPLACE_S} s "
                    "of silence).", throttle_duration_sec=1.0)
                return
            # A link that died silently never reports itself, so a new connection replaces it.
            self.close_client()
        new_sock.setblocking(False)
        self.client_sock = new_sock
        self.last_rx_s = self.now()
        self.get_logger().info("Joystick connected!")

    def close_client(self):
        if self.client_sock is not None:
            self.client_sock.close()
        self.client_sock = None
        self.rx_buffer = b""

    def read_newest_message(self):
        """Read everything waiting on the socket; return the newest complete gamepad
        message as a Joy, or None. Reading it all (not one message per tick) keeps a
        fast gamepad from building up a backlog of old commands. At most MAX_BYTES_PER_TICK
        bytes are read per call, so a flood cannot stall the node."""
        if self.client_sock is None:
            return None
        bytes_read = 0
        try:
            while bytes_read < self.MAX_BYTES_PER_TICK:
                data = self.client_sock.recv(min(4096, self.MAX_BYTES_PER_TICK - bytes_read))
                if data == b"":  # the gamepad closed the connection cleanly
                    self.get_logger().warning(
                        "Joystick disconnected. Restart computer_joystick.py on the laptop to connect again.")
                    self.close_client()
                    return None
                bytes_read += len(data)
                self.rx_buffer += data
                self.last_rx_s = self.now()
        except BlockingIOError:
            pass  # socket drained
        except OSError as error:  # e.g. connection reset
            self.get_logger().warning(
                f"Joystick connection lost ({error}). Restart computer_joystick.py on the laptop to connect again.")
            self.close_client()
            return None

        count = len(self.rx_buffer) // self.message_size
        if count == 0:
            return None
        newest = self.rx_buffer[(count - 1) * self.message_size : count * self.message_size]
        self.rx_buffer = self.rx_buffer[count * self.message_size:]
        unpack = struct.unpack(f"{self.NUM_AXES}f {self.NUM_BUTTONS}B", newest)
        axes = list(unpack[:self.NUM_AXES])
        buttons = list(unpack[self.NUM_AXES:])
        problem = self.message_problem(axes, buttons)
        if problem is not None:
            # A garbled message would be turned straight into a velocity command. Drop it, and the
            # connection with it: once the byte stream is out of step, every later message is
            # garbage too. computer_joystick.py has no reconnect loop, so someone must restart it;
            # until then nothing is published, so DiffDriveController's watchdog zeroes the
            # velocity reference.
            # LIMIT: framing errors are caught only PROBABILISTICALLY. Only the newest message is
            # checked, and a stream that is out of step by some bytes can still look valid (an
            # all-zero stream passes at ANY offset: zero axes and zero buttons are legal). Telling
            # for certain needs a header byte on every message, which is a protocol change on
            # both machines (computer_joystick.py too), not made here.
            self.get_logger().warning(
                f"Rejected a gamepad message ({problem}); dropping the connection.")
            self.close_client()
            return None
        message = Joy()
        message.axes = axes
        message.buttons = buttons
        return message

    def message_problem(self, axes, buttons):
        """None if the message looks like a gamepad's, otherwise what is wrong with it."""
        for value in axes:
            if not math.isfinite(value):
                return f"axis value {value} is not finite"
            if abs(value) > 1.0 + self.AXIS_TOLERANCE:
                return f"axis value {value} is outside [-1, 1]"
        for value in buttons:
            if value not in (0, 1):
                return f"button value {value} is not 0 or 1"
        return None


    def convert_to_twist(self, msg):

        twist = TwistStamped()

        linear_axis = 1
        angular_axis = 0
        # linear_value = msg.axes[linear_axis]
        # angular_value = msg.axes[angular_axis]
        linear_value  = expo_curve(msg.axes[linear_axis])
        angular_value = expo_curve(msg.axes[angular_axis])
        if abs(linear_value) < self.deadband:
            linear_value = 0.0
        if abs(angular_value) < self.deadband:
            angular_value = 0.0


        twist.twist.linear.x = linear_value * self.linear_scale
        twist.twist.angular.z = angular_value * self.angular_scale

        self.robot_body_vector_updated_topic.publish(twist)

        # self.get_logger().info(f"Twist: Linear- {twist.twist.linear.x:.2f}, Angular- {twist.twist.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)

    joystick = Joystick()

    rclpy.spin(joystick)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joystick.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

