"""The foxglove:= part of the launch files: start foxglove_bridge, so Foxglove on another machine can
connect to ws://<robot-ip>:8765 and plot any topic live (docs/guides/use-foxglove.md).

    from rocko_env.webserver.launch_foxglove import foxglove_arguments, foxglove_bridge_process
    declared_arguments += foxglove_arguments(default="true")
    nodes.append(foxglove_bridge_process())
"""
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def foxglove_arguments(default):
    """`default` is "true" on the robot's launch files, which always started the bridge before this
    argument existed (so a plain launch does what it did), and "false" in the simulation."""
    return [
        DeclareLaunchArgument(
            "foxglove", default_value=default,
            description="true: start foxglove_bridge (Foxglove connects to ws://<robot-ip>:8765). "
                        "false: do not, for example to rule it out when measuring loop timing."),
    ]


def foxglove_bridge_process():
    """foxglove_bridge, started exactly as the robot's launch files always started it (the bridge's
    own launch file, with its defaults: port 8765), now only when foxglove:=true."""
    return ExecuteProcess(
        cmd=["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"],
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )
