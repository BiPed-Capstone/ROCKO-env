# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    NotSubstitution,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from rocko_env.recording.launch_recording import (
    real_or_mock,
    recording_arguments,
    recorder_process,
)
from rocko_env.webserver.launch_dashboard import dashboard_arguments, dashboard_node
from rocko_env.teleop import gamepad
from rocko_env.webserver.launch_foxglove import (
    foxglove_arguments,
    foxglove_bridge_process,
)


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            # "false" keeps what actually ran before the flag worked: it used to default to "true"
            # but was ignored, so real hardware always loaded. A robot launched with no arguments
            # must get real hardware.
            default_value="false",
            description=(
                "true: swap the motor hardware plugins for mock_components/GenericSystem and "
                "skip the encoder node. A wiring check only, with no physics: commands are "
                "echoed back as states. "
                "false (default): real hardware, which is what runs on the robot."
            ),
        )
    )

    # record:=true records a rosbag and a run.yaml (docs/guides/record-a-run.md). Off by default.
    declared_arguments += recording_arguments()
    # dashboard:=false leaves the web dashboard out (docs/guides/use-the-dashboard.md). ON by default
    # here: tank mode has no balance loop for it to disturb. rocko.launch.py keeps it off, because its
    # cost beside the 100 Hz balance loop has not been measured.
    declared_arguments += dashboard_arguments(default="true")
    # foxglove:=false leaves foxglove_bridge out. It defaults to true: this launch always started it.
    declared_arguments += foxglove_arguments(default="true")
    # teleop:=false leaves the USB controller nodes out, for tests that publish cmd_vel by hand (the bench
    # test): TankTeleop.py publishes zero 50 times a second when LB is not held, which would fight them.
    declared_arguments.append(DeclareLaunchArgument(
        "teleop", default_value="true",
        description="true (default): start the USB controller nodes. false: nothing publishes cmd_vel"))
    # Caps for the USB controller (TankTeleop.py), fractions of full speed: lower them for a first
    # test, e.g. max_speed:=0.4 max_turn:=0.3. TankTeleop.py refuses a value outside [0, 1].
    declared_arguments.append(DeclareLaunchArgument(
        "max_speed", default_value="1.0", description="cap on any wheel, fraction of full (0 to 1)"))
    declared_arguments.append(DeclareLaunchArgument(
        "max_turn", default_value="1.0", description="turn stick strength, fraction of full (0 to 1)"))

    # supervised:=true is how the robot supervisor (Supervisor.py, docs/architecture/robot-supervisor.md)
    # starts this launch as a child. The base launch it runs from (rocko_robot.launch.py) already
    # runs the joy node, the dashboard and foxglove_bridge, so they are left out here; and the motion
    # controller (diffbot_base_controller) is spawned INACTIVE, so no command moves the wheels until
    # the supervisor's "enable". false (default): exactly what this launch always did, for bench tests by hand.
    declared_arguments.append(DeclareLaunchArgument(
        "supervised", default_value="false",
        description="true: started by the supervisor: no joy node, dashboard or foxglove (the base launch "
                    "has them) and diffbot_base_controller starts inactive. false (default): standalone."))

    # Initialize Arguments
    supervised = LaunchConfiguration("supervised")
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rocko_env"), "urdf", "rocko_tank_mode.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    # Declare the URDF a string. Without value_type, launch_ros tries to parse it as YAML, so a
    # stray ": " in any xacro comment makes the whole launch fail with a parse error.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rocko_env"),
            "config",
            "rocko_tank_mode_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rocko_env"), "rviz", "rocko.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    foxglove_bridge = foxglove_bridge_process()

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Two spawners, one runs: unsupervised loads and ACTIVATES the drive controller (as always);
    # supervised loads and configures it but leaves it INACTIVE (spawner --inactive), so the
    # supervisor's enable is what activates it.
    diffdrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--param-file", robot_controllers],
        condition=UnlessCondition(supervised),
    )
    diffdrive_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--param-file", robot_controllers, "--inactive"],
        condition=IfCondition(supervised),
    )

    # joint_state_broadcaster goes after the drive spawner, whichever of the two ran.
    delay_joint_state_broadcaster_after_robot_controller_spawner = [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=spawner, on_exit=[joint_state_broadcaster_spawner]))
        for spawner in (diffdrive_spawner, diffdrive_spawner_inactive)
    ]

    encoders = Node(
        package="rocko_env",
        executable="QuadEncoders.py",
        parameters=[
            {"left_a_pin": 26, "left_b_pin": 20, "right_a_pin": 19, "right_b_pin": 16}
        ],
        # Reads the real encoders over GPIO, so it only starts on the robot.
        condition=UnlessCondition(use_mock_hardware),
    )

    # TELEOP: a game controller plugged into the Pi's USB (tank mode only; balancing mode does not
    # start these). joy's game_controller_node reads it through SDL and publishes /joy;
    # TankTeleop.py turns /joy into cmd_vel (split arcade, hold LB to drive). deadzone 0.0 here
    # because TankTeleop.py applies its own (one place, and tested); autorepeat_rate is how often the
    # joy node republishes an unchanged state, which TankTeleop.py's timeout is counted in.
    # Guide: docs/guides/drive-with-usb-controller.md.
    game_controller = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[{"deadzone": 0.0, "autorepeat_rate": gamepad.JOY_AUTOREPEAT_RATE_HZ}],
        # The supervisor's base launch runs the joy node when supervised.
        condition=IfCondition(AndSubstitution(LaunchConfiguration("teleop"), NotSubstitution(supervised))),
    )
    tank_teleop = Node(
        package="rocko_env",
        executable="TankTeleop.py",
        parameters=[{
            # value_type=float: "1" typed on the command line must still be a float parameter.
            "max_speed": ParameterValue(LaunchConfiguration("max_speed"), value_type=float),
            "max_turn": ParameterValue(LaunchConfiguration("max_turn"), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # Printed first, so nobody mistakes a mock run for a live robot.
    mock_banner = LogInfo(
        msg="MOCK HARDWARE (use_mock_hardware:=true): no motors. A wiring check only.",
        condition=IfCondition(use_mock_hardware),
    )

    nodes = [
        mock_banner,
        control_node,
        robot_state_pub_node,
        diffdrive_spawner,
        diffdrive_spawner_inactive,
        *delay_joint_state_broadcaster_after_robot_controller_spawner,
        encoders,
        game_controller,
        tank_teleop,
    ]

    # Only with record:=true. "mock" or "real" goes into run.yaml so a mock run is never mistaken
    # for a measurement.
    nodes.append(
        recorder_process(
            "rocko_tank_mode.launch.py",
            real_or_mock(use_mock_hardware),
            robot_controllers,
        )
    )
    # The dashboard and foxglove_bridge run in the supervisor's base launch when supervised, so they
    # are grouped to run only when not supervised (their own dashboard:= / foxglove:= switches still apply).
    nodes.append(GroupAction(
        [
            foxglove_bridge,
            dashboard_node(
                "rocko_tank_mode.launch.py",
                real_or_mock(use_mock_hardware),
                robot_controllers,
            ),
        ],
        condition=UnlessCondition(supervised),
    ))

    return LaunchDescription(declared_arguments + nodes)
