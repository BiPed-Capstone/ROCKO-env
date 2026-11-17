"""The dashboard:= part of the launch files, in one place so all three start it the same way.

    from rocko_env.webserver.launch_dashboard import dashboard_arguments, dashboard_node
    declared_arguments += dashboard_arguments(default="false")
    nodes.append(dashboard_node("rocko.launch.py", real_or_mock(use_mock_hardware), robot_controllers))

It needs the log_root argument that recording_arguments() declares (the dashboard lists and makes
recordings in that folder), so a launch file uses both.
"""
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def dashboard_arguments(default):
    """The launch arguments about the dashboard. `default` is "false" on the robot's launch files:
    what the dashboard costs the Pi's CPU while the 100 Hz loop runs has not been measured."""
    return [
        DeclareLaunchArgument(
            "dashboard", default_value=default,
            description="true: start the web dashboard (live values and Record/Stop in a browser on "
                        "another machine: http://<robot-ip>:<dashboard_port>). No login."),
        DeclareLaunchArgument(
            "dashboard_port", default_value="8080",
            description="dashboard:=true only: the TCP port the dashboard listens on."),
    ]


def dashboard_node(launch_file, hardware, config):
    """Dashboard.py as a launch action that runs only when dashboard:=true.
    launch_file, hardware and config are what a recording started from the page writes to run.yaml
    (the same three values recorder_process() takes)."""
    return Node(
        package="rocko_env",
        executable="Dashboard.py",
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("dashboard_port"), value_type=int),
            "log_root": ParameterValue(LaunchConfiguration("log_root"), value_type=str),
            "launch": launch_file,
            "hardware": ParameterValue(hardware, value_type=str),
            "config": ParameterValue(config, value_type=str),
        }],
        output="both",
        # A recording started from the page is closed when the dashboard stops; give it the time
        # RecordRun.py needs (see launch_recording.py).
        sigterm_timeout="30",
        condition=IfCondition(LaunchConfiguration("dashboard")),
    )
