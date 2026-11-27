"""The robot's base launch: starts at boot and stays up (docs/architecture/robot-supervisor.md).

It runs what every mode shares (the supervisor, the game controller's joy node, the dashboard,
foxglove_bridge, and the optional extras: battery monitor, buzzer, status screen). The supervisor then starts the current mode's own launch file as a child process
with supervised:=true, and stops and restarts it when the mode changes. Do not start a mode's launch
file by hand while this runs: two stacks fight over the motors.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rocko_env.extras.launch_extras import extras_arguments, extras_nodes
from rocko_env.recording.launch_recording import real_or_mock, recording_arguments
from rocko_env.supervisor import modes
from rocko_env.teleop import gamepad
from rocko_env.webserver.launch_dashboard import dashboard_arguments, dashboard_node
from rocko_env.webserver.launch_foxglove import foxglove_arguments, foxglove_bridge_process

# [s] The supervisor stops the child stack on shutdown and may wait up to its stop_timeout_s (15 s)
# before SIGKILL. launch must not escalate to SIGTERM first, so wait longer than that.
SUPERVISOR_SIGTERM_TIMEOUT_S = "25"


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            # "false": a robot launched with no arguments must get real hardware.
            default_value="false",
            description=(
                "true: the mode stacks use mock_components/GenericSystem and skip the nodes that talk "
                "to real sensors. A wiring check only, with no physics: commands are echoed back as "
                "states and pitch stays 0.0. "
                "false (default): real hardware, which is what runs on the robot."
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_mode", default_value="tank",
            description="The mode whose stack starts at boot: tank or balance. The robot always "
                        "starts disabled.",
        )
    )
    # log_root (and record, ...): the dashboard's Record button needs log_root.
    declared_arguments += recording_arguments()
    # On by default: this launch is the one that starts at boot, and tank mode (the default mode) ran
    # the dashboard by default. Beside the balance loop it did not change the loop's timing measurably
    # (docs/journal/2026-11-24-loop-timing-on-the-pi.md).
    declared_arguments += dashboard_arguments(default="true")
    declared_arguments += foxglove_arguments(default="true")
    # battery:=, buzzer:=, screen:= (all true): optional, so a missing one only logs and shows on the dashboard.
    declared_arguments += extras_arguments()

    # Speed caps for each mode's teleop node, passed through by the supervisor, e.g. for a first stand test:
    #   pi/launch.sh robot tank_max_speed:=0.3 balance_max_speed:=0.02
    # Empty (the default) keeps each mode launch's own default (max_speed and max_turn in
    # rocko_tank_mode.launch.py and rocko.launch.py). A bad value becomes 0 (max_speed: no driving; max_turn: no turning).
    for mode in modes.mode_names():
        for name in modes.CAP_ARGS:
            declared_arguments.append(DeclareLaunchArgument(
                f"{mode}_{name}", default_value="",
                description=f"{mode} mode's {name} (see max_speed/max_turn in its launch file), a fraction "
                            f"in [0, 1]; empty: that launch's default; a bad value: 0"))

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    supervisor = Node(
        package="rocko_env",
        executable="Supervisor.py",
        parameters=[{
            "use_mock_hardware": ParameterValue(use_mock_hardware, value_type=bool),
            "initial_mode": ParameterValue(LaunchConfiguration("initial_mode"), value_type=str),
            "log_root": ParameterValue(LaunchConfiguration("log_root"), value_type=str),
            **{f"{mode}_{name}": ParameterValue(LaunchConfiguration(f"{mode}_{name}"), value_type=str)
               for mode in modes.mode_names() for name in modes.CAP_ARGS},
        }],
        output="both",
        sigterm_timeout=SUPERVISOR_SIGTERM_TIMEOUT_S,
    )

    # joy's game_controller_node reads the USB controller through SDL and publishes /joy. deadzone 0.0
    # because the teleop nodes apply their own (one place, tested); autorepeat_rate is how often joy
    # republishes an unchanged state, which the 0.25 s silence timeouts are counted in.
    game_controller = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[{"deadzone": 0.0, "autorepeat_rate": gamepad.JOY_AUTOREPEAT_RATE_HZ}],
    )

    mock_banner = LogInfo(
        msg="MOCK HARDWARE (use_mock_hardware:=true): no motors. A wiring check only.",
        condition=IfCondition(use_mock_hardware),
    )

    nodes = [mock_banner, supervisor, game_controller, foxglove_bridge_process()]
    # The dashboard's recordings write the config path into run.yaml; the mode's own YAML is chosen by
    # the supervisor, so name the launch instead.
    nodes.append(dashboard_node("rocko_robot.launch.py", real_or_mock(use_mock_hardware), "(per mode)"))
    nodes += extras_nodes()

    return LaunchDescription(declared_arguments + nodes)
