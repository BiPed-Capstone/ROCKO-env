"""The battery:=, buzzer:= and screen:= part of rocko_robot.launch.py: the optional extras.

    from rocko_env.extras.launch_extras import extras_arguments, extras_nodes
    declared_arguments += extras_arguments()
    nodes += extras_nodes()

Each extra is its own process, so one that crashes takes nothing else with it: a launch only ends when a node
marked on_exit=Shutdown ends, and these are not. A missing sensor is not a crash: the node keeps running, retries
its hardware itself and reports the error (health.py). A crashed extra stays down and the dashboard shows it as
NOT RUNNING until the robot launch restarts.

Why no respawn=True: tried 2026-11-27, and a Ctrl-C (or systemd stopping rocko.service, which signals every
process) let the extras exit cleanly a moment before launch began its own shutdown, launch respawned them, and the
launch never ended.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXTRA_NODES = {          # launch argument -> (executable, what it is)
    "battery": ("BatteryMonitor.py", "the battery voltage from an INA226 on its own I2C bus"),
    "buzzer": ("Buzzer.py", "sounds on boot, enable, disable and warnings, from a passive buzzer"),
    "screen": ("StatusScreen.py", "the 3.5 inch status screen on the robot"),
}


def extras_arguments():
    return [DeclareLaunchArgument(
        name, default_value="true",
        description=f"true (default): start {executable}: {what}. Optional: if its hardware is missing it logs "
                    f"the error, shows it on the dashboard, and the robot runs as usual.")
        for name, (executable, what) in EXTRA_NODES.items()]


def extras_nodes():
    # The settings a person may change on the robot (device paths, screen rotation, touch axes): extras.yaml.
    settings = os.path.join(get_package_share_directory("rocko_env"), "extras", "extras.yaml")
    return [Node(package="rocko_env", executable=executable, output="both", parameters=[settings],
                 condition=IfCondition(LaunchConfiguration(name)))
            for name, (executable, _) in EXTRA_NODES.items()]
