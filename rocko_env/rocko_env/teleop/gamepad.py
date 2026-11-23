"""What every ROCKO teleop node needs to know about a game controller, in ONE place.

Imported by tank mode's teleop node (controllers/tank_teleop/TankTeleop.py) and the dashboard
(webserver/Dashboard.py), and meant for balancing mode's teleop when it gets a controller of its own.
Nothing here knows which mode is running: speeds, scales and which button is the deadman belong to
the node that uses them.

WHERE THE NUMBERS COME FROM. The controller is read by ROS's own `joy` package, node
`game_controller_node` (package ros-jazzy-joy). It uses SDL's GameController layer, which looks the
controller up in SDL's mapping database and reports every supported controller in the same standard
layout, so "left stick Y" is axis 1 whether the pad is an Xbox, an 8BitDo or a PlayStation one. It
publishes sensor_msgs/Joy on /joy. The index lists below are that node's (joy 3.3.0, README and
src/game_controller.cpp of ros-drivers/joystick_drivers).

SIGNS (ROS convention; the joy node inverts SDL's raw values to get them): a stick pushed UP or LEFT
reads +1, pushed DOWN or RIGHT reads -1. That is the robot's own sense: forward is +x and a left turn
is +z, so a stick value can be used as a command without flipping it. NOT seen on the robot's
controller yet: `ros2 topic echo /joy` on the Pi is the check (docs/guides/drive-with-usb-controller.md).
"""
import math

# --- the /joy layout of joy's game_controller_node ----------------------------------------------
AXIS_NAMES = ("left_x", "left_y", "right_x", "right_y", "trigger_left", "trigger_right")
AXIS_LEFT_X = AXIS_NAMES.index("left_x")      # 0; +1 = pushed left
AXIS_LEFT_Y = AXIS_NAMES.index("left_y")      # 1; +1 = pushed up (forward)
AXIS_RIGHT_X = AXIS_NAMES.index("right_x")    # 2; +1 = pushed left
AXIS_RIGHT_Y = AXIS_NAMES.index("right_y")    # 3; +1 = pushed up
# Triggers (4, 5): 0 at rest; the joy node applies the same inversion to them, so a full press is
# expected to read about -1. Inferred from its code, not seen on hardware. Not used for driving.

BUTTON_NAMES = ("a", "b", "x", "y", "back", "guide", "start", "left_stick", "right_stick",
                "left_bumper", "right_bumper", "dpad_up", "dpad_down", "dpad_left", "dpad_right",
                "misc1", "paddle1", "paddle2", "paddle3", "paddle4", "touchpad")
BUTTON_LEFT_BUMPER = BUTTON_NAMES.index("left_bumper")    # 9  (LB, L1)
BUTTON_RIGHT_BUMPER = BUTTON_NAMES.index("right_bumper")  # 10 (RB, R1)

# --- how often the joy node publishes, and when silence means "the controller is gone" -----------
# Passed to game_controller_node as autorepeat_rate (the tank launch file reads it from here): while a
# controller is connected it republishes its unchanged state this often. When the controller is
# unplugged it publishes NOTHING (it does not send zeros), so a teleop node must notice the silence.
JOY_AUTOREPEAT_RATE_HZ = 20.0
JOY_PERIOD_S = 1.0 / JOY_AUTOREPEAT_RATE_HZ   # [s] 0.05
MISSED_MESSAGES_BEFORE_GONE = 5               # one late message is not a lost controller; five is
JOY_TIMEOUT_S = MISSED_MESSAGES_BEFORE_GONE * JOY_PERIOD_S  # [s] 0.25


def clamp(value, low=-1.0, high=1.0):
    return max(low, min(high, value))


def shape(raw, deadzone, expo, min_output=0.0):
    """One stick axis in [-1, 1] -> a command in [-1, 1], same sign.
    1. Deadzone: |raw| <= deadzone is 0 (a stick at rest is rarely exactly 0).
    2. Rescale what is left to 0..1, so the command starts at 0 at the deadzone's edge instead of
       jumping to `deadzone` (a jump would be a kick to the motors).
    3. Expo: (1 - expo) * x + expo * x^3. At x = 1 it is still 1, so expo never costs top speed; it
       only spreads the slow speeds over more of the stick's travel.
    4. Lift by min_output: min_output + (1 - min_output) * curve, still 1 at full stick.
    A reading that is not a number (NaN, inf) is 0: clamp() alone would turn NaN into +1, because
    min(1.0, nan) is 1.0 in Python, and a broken reading must never become full speed.
    deadzone must be below 1, or step 2 divides by zero (the caller checks its parameter)."""
    if not math.isfinite(raw):
        return 0.0
    raw = clamp(raw)
    magnitude = abs(raw)
    if magnitude <= deadzone:
        return 0.0
    x = (magnitude - deadzone) / (1.0 - deadzone)
    curved = (1.0 - expo) * x + expo * x ** 3
    lifted = min_output + (1.0 - min_output) * curved
    return lifted if raw > 0 else -lifted


def mix(forward, turn_left, max_speed=1.0, max_turn=1.0):
    """Combine forward and turn (each in [-1, 1]; forward > 0 drives forward, turn_left > 0 turns
    LEFT, which makes the right wheel faster) so that no wheel is asked for more than max_speed of
    full wheel speed. Returns (forward, turn_left), still as fractions of full.
    Each wheel's speed, as a fraction of full, is forward - turn_left (left wheel) and
    forward + turn_left (right wheel). That holds only if the caller scales turning so that a full
    turn alone gives the same wheel speed as a full forward (TankTeleop.py does). The steps:
    1. max_turn weakens the turn stick first, so it is part of what the wheels must fit.
    2. Full forward plus some turn would ask one wheel for more than 1: the motor would clip at
       100 % and the turn would be lost. So when |forward| + |turn| > 1 both are divided by that sum,
       which keeps the turn and the direction of travel.
    3. max_speed scales both last, so it caps every wheel."""
    turn = turn_left * max_turn
    total = abs(forward) + abs(turn)
    if total > 1.0:
        forward, turn = forward / total, turn / total
    return forward * max_speed, turn * max_speed


class DeadmanSwitch:
    """Hold-a-button-to-drive, with a re-arm rule.

    update() returns True (driving allowed) only while the controller is heard from AND the button
    is held AND the button has been seen RELEASED since the controller was last silent (or since
    start-up). Why the re-arm: the joy node does not reset its buttons when a controller is unplugged
    and plugged back in, so a button held at the moment of the unplug can still read 1 afterwards. It
    also means a robot started with the button already held does not drive off."""

    def __init__(self):
        self.armed = False  # True once the button has been seen released while the controller was heard

    def update(self, controller_heard, button_held):
        if not controller_heard:
            self.armed = False
            return False
        if not button_held:
            self.armed = True
            return False
        return self.armed
