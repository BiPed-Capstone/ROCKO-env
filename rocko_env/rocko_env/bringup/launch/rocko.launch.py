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
    foxglove_bridge = ExecuteProcess(cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"])
    
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", robot_controllers,
                   "--activate-as-group",
                    "left_add_feedforward_controller", 
                    "right_add_feedforward_controller",
                    "left_pitch_pid_controller",
                    "right_pitch_pid_controller",
                    "left_velocity_pid_controller",
                    "right_velocity_pid_controller",
                   ],
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
    
    diffdrive_controller = Node(
        package="rocko_env",
        executable="DiffDriveController.py",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        # foxglove_bridge,
        gyro,
        left_relative_encoder,
        right_relative_encoder,
        controller_spawner,
        diffdrive_controller
    ]

    return LaunchDescription(declared_arguments + nodes)