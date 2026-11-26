"""The one table of robot modes. Adding a mode is adding a row here (docs/architecture/robot-supervisor.md).

Plain Python, no ROS, so the state machine and its test can import it on a laptop.
"""

# Nodes (by ROS node name) that every mode's launch starts. real_only=True: the node talks to real
# sensors (I2C/GPIO), so it is not started, and not expected, with use_mock_hardware:=true.
# No node needs it since 2026-11-24: the IMU and the encoders are read inside the hardware plugins
# (finding C-1), so a dead sensor shows up as a hardware error in controller_manager, not a missing node.
# Kept for the next sensor that has to be a separate node.
# Each name is the one the node gives itself in its source (see the comments in MODES below).


def _node(name, real_only=False):
    return {"name": name, "real_only": real_only}


MODES = {
    "tank": {
        "launch_file": "rocko_tank_mode.launch.py",
        # Active as soon as the stack is up, whether or not the robot is enabled.
        "always_active": ["joint_state_broadcaster"],
        # Inactive when disabled, active when enabled. Enable = activate these together.
        "motion": ["diffbot_base_controller"],
        # Disabling tank mode just stops the wheels, so losing the game controller disables it.
        "falls_when_disabled": False,
        "nodes": [
            _node("controller_manager"),
            _node("robot_state_publisher"),
            _node("tank_teleop"),                   # TankTeleop.py
        ],
    },
    "balance": {
        "launch_file": "rocko.launch.py",
        # Supervised balance spawns it so /dynamic_joint_states carries the pitch the enable rule needs.
        "always_active": ["joint_state_broadcaster"],
        "motion": [
            "left_add_feedforward_controller",
            "right_add_feedforward_controller",
            "pitch_pid_controller",
            "velocity_pid_controller",
        ],
        # Disabling balance mode means the robot FALLS: the cascade is what holds it up. So losing the game
        # controller or a helper node must NOT disable it. The safe response to lost input is a zero velocity
        # command (finding C-3), which BalanceTeleop.py and DiffDriveController.py's watchdog already give.
        # The supervisor still disables when a cascade controller itself is gone (nothing is holding it up).
        "falls_when_disabled": True,
        "nodes": [
            _node("controller_manager"),
            _node("robot_state_publisher"),
            _node("diff_drive_controller"),          # DiffDriveController.py
            _node("balance_teleop"),                 # the name BalanceTeleop.py gives itself
        ],
    },
}

# Nodes the supervisor's own base launch (rocko_robot.launch.py) starts, shown on the dashboard
# but not part of any stack: losing one is not a stack fault.
BASE_NODES = ["supervisor", "game_controller_node"]

DEFAULT_MODE = "tank"

# Speed caps the supervisor can pass to a mode's launch, as `name:=value`. Every mode's launch declares
# both (rocko_tank_mode.launch.py, rocko.launch.py) and gives them to its teleop node. Each is a unitless
# fraction in [0, 1], not m/s: max_speed of full wheel speed (tank) or of 10 m/s forward (balance),
# max_turn the turn stick's strength. The defaults live in those launch files, not here: an empty value
# here means "not set, use the launch file's default".
CAP_ARGS = ["max_speed", "max_turn"]


def mode_names():
    return list(MODES)


def launch_file(mode):
    return MODES[mode]["launch_file"]


def always_active(mode):
    return list(MODES[mode]["always_active"])


def motion_controllers(mode):
    return list(MODES[mode]["motion"])


def falls_when_disabled(mode):
    return MODES[mode]["falls_when_disabled"]


def cap_problem(text):
    """Why `text` is not a usable cap (None if it is): empty means "not set" and is fine. A bad value
    must not reach the launch, where a word that is not a number fails the whole stack."""
    if text == "":
        return None
    try:
        value = float(text)
    except ValueError:
        return f"'{text}' is not a number"
    if not 0.0 <= value <= 1.0:   # also false for NaN
        return f"{text} is not a fraction in [0, 1]"
    return None


def checked_caps(texts):
    """{name: text} -> (caps for launch_command, error messages). An empty value is left out. A bad one
    becomes "0" (a bad max_speed: no driving; a bad max_turn: no turning) until it is fixed. Dropping it
    instead would leave the launch file's default, and a typo such as 0,3 would give tank mode full speed."""
    caps, errors = {}, []
    for name in CAP_ARGS:
        text = texts.get(name, "").strip()
        problem = cap_problem(text)
        if problem:
            caps[name] = "0"
            errors.append(f"{name}: {problem}; using 0 until it is fixed")
        elif text:
            caps[name] = text
    return caps, errors


def launch_command(mode, use_mock_hardware, log_root="", caps=None):
    """The `ros2 launch` command that starts `mode`'s stack under the supervisor. caps: {"max_speed": "0.3"};
    a missing or empty value is left out, so the launch file's default applies. Check values with
    cap_problem() first."""
    cmd = ["ros2", "launch", "rocko_env", launch_file(mode), "supervised:=true",
           f"use_mock_hardware:={'true' if use_mock_hardware else 'false'}"]
    if log_root:
        cmd.append(f"log_root:={log_root}")
    for name in CAP_ARGS:
        value = (caps or {}).get(name, "")
        if value != "":
            cmd.append(f"{name}:={value}")
    return cmd


def expected_nodes(mode, use_mock_hardware):
    """Names of the nodes that must be alive for this mode's stack to be 'ready'."""
    return [n["name"] for n in MODES[mode]["nodes"]
            if not (use_mock_hardware and n["real_only"])]
