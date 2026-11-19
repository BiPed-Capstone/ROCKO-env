# Sets up ONE terminal on the Pi to run ROCKO. SOURCE this file, do not run it:
#
#     source pi/rocko_env.sh
#
# (Running it would set up a shell that exits at once.) It does what the old hand-written helper did:
# ROS, the Python virtual environment, this workspace's build, ROS_DOMAIN_ID, then cd to the workspace.
#
# You normally do not source it yourself. pi/rocko_shell.sh, pi/rocko_run.sh and pi/launch.sh source it
# for you AS ROOT, which is what the robot needs: the motor plugin uses WiringPi to drive the GPIO pins,
# and that only works as root. Source it directly only for things that need no root, such as
# building (`./build_ros.sh`).
#
# Settings you may override before sourcing (the defaults are what the robot uses):
#   ROCKO_WS         the colcon workspace: where install/ is and where the shell ends up.
#                    Default: the folder this repo is checked out in (the workspace IS the checkout on
#                    the Pi: there is no ~/ros2_ws).
#   ROCKO_VENV       the Python virtual environment. Default: ~/.venv
#   ROCKO_DOMAIN_ID  ROS_DOMAIN_ID. Default: 17. Every terminal on the robot must use the same number,
#                    or ros2 commands in one terminal cannot see the launch running in another.
#   ROCKO_ROS_SETUP  ROS's setup file. Default: /opt/ros/jazzy/setup.bash

ROCKO_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROCKO_WS="${ROCKO_WS:-$ROCKO_REPO}"
ROCKO_VENV="${ROCKO_VENV:-$HOME/.venv}"
ROCKO_DOMAIN_ID="${ROCKO_DOMAIN_ID:-17}"
ROCKO_ROS_SETUP="${ROCKO_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

# ROS first: its setup file changes PATH and PYTHONPATH, and the workspace overlay needs it.
# shellcheck disable=SC1090
source "$ROCKO_ROS_SETUP" || echo "rocko_env: ERROR: could not source $ROCKO_ROS_SETUP" >&2

# A missing venv or build is reported, not fatal: the first thing you do on a fresh checkout is build,
# and that needs neither.
if [ -f "$ROCKO_VENV/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "$ROCKO_VENV/bin/activate"
else
  echo "rocko_env: note: no Python venv at $ROCKO_VENV (set ROCKO_VENV if yours is elsewhere)" >&2
fi
if [ -f "$ROCKO_WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ROCKO_WS/install/setup.bash"
else
  echo "rocko_env: note: $ROCKO_WS/install/setup.bash not found: build first with ./build_ros.sh" >&2
fi

export ROS_DOMAIN_ID="$ROCKO_DOMAIN_ID"
cd "$ROCKO_WS" || echo "rocko_env: ERROR: workspace $ROCKO_WS not found" >&2

echo "rocko_env: user=$(id -un) domain=$ROS_DOMAIN_ID workspace=$PWD" >&2
