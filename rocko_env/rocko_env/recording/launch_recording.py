"""The record:=true part of the launch files, in one place so all three record the same way.

Used by rocko.launch.py, rocko_tank_mode.launch.py and rocko_sim.launch.py:

    from rocko_env.recording.launch_recording import recording_arguments, recorder_process
    declared_arguments += recording_arguments()
    nodes.append(recorder_process("rocko.launch.py", real_or_mock(use_mock_hardware), robot_controllers))
"""
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def recording_arguments():
    """The launch arguments about recording. record defaults to false: a plain launch writes nothing."""
    return [
        DeclareLaunchArgument(
            "record", default_value="false",
            description="true: record a rosbag (MCAP) and a run.yaml describing the run, under log_root."),
        DeclareLaunchArgument(
            "record_label", default_value="run",
            description="record:=true only: a short name for the test; it becomes part of the folder name."),
        DeclareLaunchArgument(
            "battery_v", default_value="",
            description="record:=true only: battery voltage at the start [V], written to run.yaml."),
        DeclareLaunchArgument(
            "surface", default_value="",
            description="record:=true only: what the robot stands on (carpet, tile, a stand), written to run.yaml."),
        DeclareLaunchArgument(
            "log_root", default_value="",
            description="record:=true only: the folder each run's folder is made in. Empty (the default) "
                        "= RecordRun.py's DEFAULT_LOG_ROOT, ~/rocko_logs."),
    ]


def real_or_mock(use_mock_hardware):
    """The word for run.yaml's `hardware` field, "mock" or "real", from the use_mock_hardware launch
    argument. A PythonExpression is evaluated when the launch starts: the argument's text is put
    between the quotes, giving for example  'mock' if 'true'.lower() in ('true', '1') else 'real'.
    "True" and "1" count as true because the xacro that picks the hardware accepts them too."""
    return PythonExpression(["'mock' if '", use_mock_hardware, "'.lower() in ('true', '1') else 'real'"])


def recorder_process(launch_file, hardware, config):
    """RecordRun.py as a launch action that runs only when record:=true.

    launch_file: this launch file's name, for run.yaml.
    hardware:    "real", "mock" or "sim" (a string or a substitution).
    config:      the controllers YAML this launch loads (a substitution for its path)."""
    # launch_ros's Node runs RecordRun.py ITSELF, so launch's stop signal reaches it. Started as
    # `ros2 run rocko_env RecordRun.py` it did not: launch signals only the process it started
    # (`ros2 run`), which does not pass the signal on, so the recorder kept recording with the bag
    # open after the launch had gone (found in review, shown by verify_tools.sh).
    # RecordRun.py is not a ROS node; it ignores the --ros-args that Node adds.
    return Node(
        package="rocko_env",
        executable="RecordRun.py",
        arguments=[
            "--label", LaunchConfiguration("record_label"),
            "--log-root", LaunchConfiguration("log_root"),
            "--launch", launch_file,
            "--hardware", hardware,
            "--config", config,
            "--battery-v", LaunchConfiguration("battery_v"),
            "--surface", LaunchConfiguration("surface"),
        ],
        output="both",
        # [s] how long launch waits after SIGINT before it uses SIGTERM. Longer than the 15 s
        # RecordRun.py gives `ros2 bag record` to close the file (launch's default is 5 s).
        sigterm_timeout="20",
        condition=IfCondition(LaunchConfiguration("record")),
    )
