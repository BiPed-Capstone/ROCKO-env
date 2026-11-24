# The robot's state, published by the supervisor on /rocko/state.
# Design: docs/architecture/robot-supervisor.md
builtin_interfaces/Time stamp
string mode            # "tank" | "balance"
bool enabled
string phase           # "starting" | "ready" | "stopping" | "fault"
string requested_mode  # the mode being started during a change, else ""
string last_source     # "gamepad" | "dashboard" | "supervisor" | "boot"
string message         # reason for the last refusal or fault, "" if none
string[] available_modes
NodeStatus[] nodes
ControllerStatus[] controllers
