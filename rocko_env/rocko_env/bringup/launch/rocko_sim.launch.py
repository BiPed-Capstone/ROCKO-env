# Launch the SIMULATED robot (Tier 3): the real controllers against rocko_env/SimPlant.
#
#   ros2 launch rocko_env rocko_sim.launch.py                         # stand still, teleop nodes on
#   ros2 launch rocko_env rocko_sim.launch.py initial_pitch_deg:=2.0  # start leaning 2 deg
#   ros2 launch rocko_env rocko_sim.launch.py teleop:=false           # a script publishes references
#
# Modelled on rocko.launch.py, which drives the real robot and is not changed. What differs:
#   - the robot description has use_sim_plant:=true, so ros2_control loads the SimPlant plugin
#     instead of the motor and IMU plugins;
#   - the controller gains come from rocko_sim_controllers.yaml (sim gains live only there);
#   - joint_state_broadcaster is spawned first, so pitch and wheel velocity can be watched on
#     /dynamic_joint_states before the robot is released; the four cascade controllers are spawned
#     as one group, exactly as on the robot, once it is running;
#   - no ICM20948.py / QuadEncoders.py (the plugin is the sensor) and no foxglove_bridge.
#
# THE SIMULATOR'S PHYSICAL PARAMETERS ARE ESTIMATES, NOT MEASUREMENTS. A result here is a result
# about a model (docs/architecture/sim-plant.md).

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Simulator settings that can be set from the command line, passed straight to the xacro (and from
# there to SimPlant as <param>s). Each default MUST equal the xacro arg's default in
# description/urdf/rocko.urdf.xacro (verify_urdf.sh runs sim/check_sim_config.py, which fails if
# one differs). Scenario and fault settings are all OFF by default; times count
# from the moment the robot is released (when the controllers first command both wheels).
# The physical estimates (masses, motor figures, ...) are not listed here: change them in the xacro.
SIM_ARGUMENTS = {
    # name: (default, description)
    "initial_pitch_deg": ("0.0", "Starting lean [deg], as the controllers will read it."),
    "disable_torque": ("false", "true: the motors produce no torque (the robot must fall)."),
    "imu_fault_start_s": ("0.0", "Start of an injected IMU outage [s after release]."),
    "imu_fault_duration_s": ("0.0", "Length of the IMU outage [s]; 0 = none."),
    "impulse_time_s": ("-1.0", "When a push is applied [s after release]; negative = never."),
    "impulse_angular_velocity_deg_per_s": ("0.0", "Size of the push: added once to the lean rate [deg/s]."),
    "pitch_sign": ("-1", "Assumption: -1 = a positive wheel command makes the pitch go up."),
    "encoder_sign": ("1", "Assumption: 1 = the encoder reads positive when a positive command drives the robot."),
    "imu_delay_ms": ("20", "IMU delay [ms] (an estimate)."),
    "model_encoder_node": ("true", "true: copy QuadEncoders.py, including its halved speed (finding U-2)."),
}


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "teleop",
            default_value="true",
            description=(
                "true: start Joystick.py and DiffDriveController.py as on the robot (with no gamepad "
                "connected, DiffDriveController keeps sending a zero velocity reference). false: "
                "start neither; whoever runs the scenario publishes the references."
            ),
        ),
        # The teleop watchdog's timeout (finding C-3), passed to DiffDriveController.py. The default
        # MUST equal that node's own default for command_timeout_s (0.2 s), so a plain launch tests
        # what the robot runs. It is an argument so verify_sim.sh's watchdog check can be shown to
        # fail: with a huge timeout the watchdog never fires.
        DeclareLaunchArgument(
            "command_timeout_s",
            default_value="0.2",
            description=(
                "teleop:=true only: seconds without a gamepad command before DiffDriveController "
                "zeroes the velocity reference (finding C-3). Default = the node's default."
            ),
        ),
    ]
    for name, (default, description) in SIM_ARGUMENTS.items():
        declared_arguments.append(DeclareLaunchArgument(name, default_value=default, description=description))

    teleop = LaunchConfiguration("teleop")

    # Get URDF via xacro, with the simulator selected and every SIM_ARGUMENTS value passed on.
    xacro_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("rocko_env"), "urdf", "rocko.urdf.xacro"]),
        " use_sim_plant:=true",
    ]
    for name in SIM_ARGUMENTS:
        xacro_command += [f" {name}:=", LaunchConfiguration(name)]
    # A string parameter: without value_type, launch_ros parses the URDF as YAML and a ": " in any
    # xacro comment breaks the launch.
    robot_description = {
        "robot_description": ParameterValue(Command(xacro_command), value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("rocko_env"), "config", "rocko_sim_controllers.yaml"]
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

    # First the broadcaster, so /dynamic_joint_states (pitch, wheel velocity) is flowing before the
    # cascade starts commanding the wheels and SimPlant releases the robot.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    # The cascade, exactly as rocko.launch.py spawns it on the robot.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--param-file", robot_controllers,
                   "--activate-as-group",
                   "left_add_feedforward_controller",
                   "right_add_feedforward_controller",
                   "pitch_pid_controller",
                   "velocity_pid_controller",
                   ],
    )
    cascade_after_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[controller_spawner])
    )

    diffdrive_controller = Node(
        package="rocko_env",
        executable="DiffDriveController.py",
        # value_type=float: "1000" would otherwise arrive as an integer, which the node's
        # declare_parameter (default 0.2, a float) refuses.
        parameters=[{"command_timeout_s": ParameterValue(
            LaunchConfiguration("command_timeout_s"), value_type=float)}],
        condition=IfCondition(teleop),
    )
    joystick = Node(
        package="rocko_env",
        executable="Joystick.py",
        condition=IfCondition(teleop),
    )

    # Printed first, so nobody mistakes a simulation for the robot.
    sim_banner = LogInfo(
        msg="SIMULATED ROBOT (rocko_env/SimPlant): a model with ESTIMATED, unmeasured parameters. "
            "No motors, no IMU. Results are about the model, not the robot."
    )

    nodes = [
        sim_banner,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        cascade_after_broadcaster,
        joystick,
        diffdrive_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
