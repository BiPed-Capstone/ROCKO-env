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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
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
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rocko_env"),
            "config",
            "rocko_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rocko_env"), "rviz", "rocko.rviz"]
    )

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    #     remappings=[
    #         ("/diffbot_base_controller/cmd_vel", "/cmd_vel"),
    #     ],
    # )
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
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diffdrive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--param-file", robot_controllers],
    )
    
    left_balancing_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_balancing_pid_controller", "--param-file", robot_controllers],
    )
    
    left_velocity_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_velocity_pid_controller", "--param-file", robot_controllers],
    )
    
    delay_left_velocity_controller_spawner_after_balancing_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=left_balancing_pid_controller_spawner,
                on_exit=[left_velocity_pid_controller_spawner],
            )
        )
    )
    
    right_balancing_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_balancing_pid_controller", "--param-file", robot_controllers],
    )
    
    right_velocity_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_velocity_pid_controller", "--param-file", robot_controllers],
    )
    
    delay_right_after_left_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=left_velocity_pid_controller_spawner,
                on_exit=[right_balancing_pid_controller_spawner],
            )
        )
    )
    
    delay_right_velocity_controller_spawner_after_balancing_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=right_balancing_pid_controller_spawner,
                on_exit=[right_velocity_pid_controller_spawner],
            )
        )
    )
    
    delay_diffdrive_after_pid_controller_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=right_velocity_pid_controller_spawner,
                on_exit=[diffdrive_spawner],
            )
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_balancing_pid_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    gyro = Node(
        package="rocko_env",
        executable="ICM20948.py",
    )

    left_relative_encoder = Node(
        package="rocko_env",
        executable="QuadEncoder.py",
        name="left_wheel_joint_encoder",
        parameters=[{
            "service_name": "left_wheel_joint_encoder_data",
            "a_pin": 26,
            "b_pin": 20
        }]
    )

    right_relative_encoder = Node(
        package="rocko_env",
        executable="QuadEncoder.py",
        name="right_wheel_joint_encoder",
        parameters=[{
            "service_name": "right_wheel_joint_encoder_data",
            "a_pin": 19,
            "b_pin": 16
        }]
    )
    
    balancing_controller = Node(
        package="rocko_env",
        executable="BalancingController.py",
    )

    ratelimiting_controller = Node(
        package="rocko_env",
        executable="RateLimitingController.py",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        foxglove_bridge,
        left_balancing_pid_controller_spawner,
        delay_left_velocity_controller_spawner_after_balancing_controller_spawner,
        right_balancing_pid_controller_spawner,
        delay_right_velocity_controller_spawner_after_balancing_controller_spawner,
        # delay_diffdrive_after_pid_controller_spawner,
        balancing_controller,
        ratelimiting_controller,
        gyro,
        left_relative_encoder,
        right_relative_encoder,
    ]

    return LaunchDescription(declared_arguments + nodes)