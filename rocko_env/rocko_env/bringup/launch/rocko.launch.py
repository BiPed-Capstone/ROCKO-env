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
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    AndSubstitution, Command, FindExecutable, NotSubstitution, PathJoinSubstitution, LaunchConfiguration)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from rocko_env.recording.launch_recording import real_or_mock, recording_arguments, recorder_process
from rocko_env.teleop import gamepad
from rocko_env.webserver.launch_dashboard import dashboard_arguments, dashboard_node
from rocko_env.webserver.launch_foxglove import foxglove_arguments, foxglove_bridge_process


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
                "true: swap the motor and IMU hardware plugins for mock_components/GenericSystem "
                "and skip the nodes that talk to real sensors. A wiring check only, with no "
                "physics: commands are echoed back as states and pitch stays 0.0. "
                "false (default): real hardware, which is what runs on the robot."
            ),
        )
    )

    # record:=true records a rosbag and a run.yaml (docs/guides/record-a-run.md). Off by default.
    declared_arguments += recording_arguments()
    # dashboard:=true starts the web dashboard (docs/guides/use-the-dashboard.md). Off by default on
    # the robot: launch_dashboard.py says why.
    declared_arguments += dashboard_arguments(default="false")
    # foxglove:=false leaves foxglove_bridge out. It defaults to true: this launch always started it.
    declared_arguments += foxglove_arguments(default="true")

    # teleop:=false leaves the USB controller nodes out, for tests that publish cmd_vel by hand.
    declared_arguments.append(DeclareLaunchArgument(
        "teleop", default_value="true",
        description="true (default): start the USB controller nodes. false: nothing publishes cmd_vel"))
    # Caps for the USB controller (BalanceTeleop.py), fractions of full speed (0 to 1). Low on purpose:
    # balancing mode has never balanced on the robot, so a first test should be slow.
    declared_arguments.append(DeclareLaunchArgument(
        "max_speed", default_value="0.05", description="cap on forward speed, fraction of full (10 m/s): 0.05 = 0.5 m/s. Untested starting point"))
    declared_arguments.append(DeclareLaunchArgument(
        "max_turn", default_value="0.2", description="turn stick strength, fraction of full (0 to 1); also scaled by max_speed"))
    # supervised:=true is how the robot supervisor (Supervisor.py, docs/architecture/robot-supervisor.md)
    # starts this launch as a child. Its base launch (rocko_robot.launch.py) already runs the joy node,
    # the dashboard and foxglove_bridge, so they are left out here; and the four cascade controllers are
    # spawned INACTIVE, so nothing commands the wheels until the supervisor's "enable".
    # false (default): what this launch always did, for bench tests by hand.
    declared_arguments.append(DeclareLaunchArgument(
        "supervised", default_value="false",
        description="true: started by the supervisor: no joy node, dashboard or foxglove (the base launch "
                    "has them) and the cascade controllers start inactive. false (default): standalone."))

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
                [FindPackageShare("rocko_env"), "urdf", "rocko.urdf.xacro"]
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
            "rocko_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both"
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    foxglove_bridge = foxglove_bridge_process()
    
    # The four cascade controllers, in this order, as one group. Two spawners, one runs. Unsupervised:
    # --activate-as-group activates them together, so the chain never runs half-up. Supervised: --inactive,
    # without --activate-as-group (nothing is activated, so there is no group to activate); the
    # supervisor later activates all four in one switch_controller call, which is the same "as a group".
    cascade_controllers = [
        "left_add_feedforward_controller",
        "right_add_feedforward_controller",
        "pitch_pid_controller",
        "velocity_pid_controller",
    ]
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", robot_controllers, "--activate-as-group", *cascade_controllers],
        condition=UnlessCondition(supervised),
    )
    controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", robot_controllers, "--inactive", *cascade_controllers],
        condition=IfCondition(supervised),
    )

    # Supervised only: the supervisor reads pitch from /dynamic_joint_states to refuse "enable" when the robot
    # is not upright, and only joint_state_broadcaster publishes that topic. The controllers YAML already
    # declares it; nothing in balance mode spawned it. It reads state interfaces only (no command interface).
    # Started after the cascade spawner has finished, like tank mode's, so two spawners never switch at once.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers],
    )
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner_inactive, on_exit=[joint_state_broadcaster_spawner]),
        condition=IfCondition(supervised),
    )

    # No sensor nodes: the ICM20948 hardware plugin reads the BNO055 over I2C and the motor plugins count
    # their encoders' edges themselves (finding C-1; docs/architecture/imu-i2c-read.md and
    # docs/architecture/encoders-in-cpp.md). The pins are hardware parameters in the ros2_control xacro.

    diffdrive_controller = Node(
        package="rocko_env",
        executable="DiffDriveController.py",
    )

    # TELEOP: a game controller plugged into the Pi's USB. joy's game_controller_node reads it and
    # publishes /joy; BalanceTeleop.py turns /joy into cmd_vel for DiffDriveController.py. deadzone 0.0
    # here because BalanceTeleop.py applies its own; autorepeat_rate is the joy node's republish rate, which the teleop's
    # silence timeout is counted in. The supervisor's base launch runs the joy node when supervised.
    game_controller = Node(
        package="joy",
        executable="game_controller_node",
        parameters=[{"deadzone": 0.0, "autorepeat_rate": gamepad.JOY_AUTOREPEAT_RATE_HZ}],
        condition=IfCondition(AndSubstitution(LaunchConfiguration("teleop"), NotSubstitution(supervised))),
    )
    balance_teleop = Node(
        package="rocko_env",
        executable="BalanceTeleop.py",
        parameters=[{
            # value_type=float: "1" typed on the command line must still be a float parameter.
            "max_speed": ParameterValue(LaunchConfiguration("max_speed"), value_type=float),
            "max_turn": ParameterValue(LaunchConfiguration("max_turn"), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # Printed first, so nobody mistakes a mock run for a live robot.
    mock_banner = LogInfo(
        msg="MOCK HARDWARE (use_mock_hardware:=true): no motors, no IMU, pitch stays 0.0. "
            "A wiring check only.",
        condition=IfCondition(use_mock_hardware),
    )

    nodes = [
        mock_banner,
        control_node,
        robot_state_pub_node,
        game_controller,
        balance_teleop,
        controller_spawner,
        controller_spawner_inactive,
        delay_joint_state_broadcaster,
        diffdrive_controller,
    ]

    # Only with record:=true. "mock" or "real" goes into run.yaml so a mock run is never mistaken
    # for a measurement.
    nodes.append(recorder_process("rocko.launch.py", real_or_mock(use_mock_hardware), robot_controllers))
    # The dashboard and foxglove_bridge run in the supervisor's base launch when supervised, so they are
    # grouped to run only when not supervised (their own dashboard:= / foxglove:= switches still apply).
    nodes.append(GroupAction(
        [foxglove_bridge, dashboard_node("rocko.launch.py", real_or_mock(use_mock_hardware), robot_controllers)],
        condition=UnlessCondition(supervised),
    ))

    return LaunchDescription(declared_arguments + nodes)