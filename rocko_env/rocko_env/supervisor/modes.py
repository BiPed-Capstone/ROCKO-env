"""The one table of robot modes. Adding a mode is adding a row here (docs/architecture/robot-supervisor.md).

Plain Python, no ROS, so the state machine and its test can import it on a laptop.
"""

# Nodes (by ROS node name) that every mode's launch starts. real_only=True: the node talks to real
# sensors (I2C/GPIO), so it is not started, and not expected, with use_mock_hardware:=true.
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
            _node("node", real_only=True),          # QuadEncoders.py (real encoders), which names itself "node"
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
            _node("icm20948_node", real_only=True),  # ICM20948.py (the BNO055 IMU)
            _node("node", real_only=True),           # QuadEncoders.py (real encoders), which names itself "node"
            _node("diff_drive_controller"),          # DiffDriveController.py
            _node("balance_teleop"),                 # the name BalanceTeleop.py gives itself
        ],
    },
}

# Nodes the supervisor's own base launch (rocko_robot.launch.py) starts, shown on the dashboard
# but not part of any stack: losing one is not a stack fault.
BASE_NODES = ["supervisor", "game_controller_node"]

DEFAULT_MODE = "tank"


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


def expected_nodes(mode, use_mock_hardware):
    """Names of the nodes that must be alive for this mode's stack to be 'ready'."""
    return [n["name"] for n in MODES[mode]["nodes"]
            if not (use_mock_hardware and n["real_only"])]
