#!/usr/bin/env bash
# Builds the ROS packages
# Intended to be ran inside the Docker container or on the robot
yellow='\e[0;33m'
white='\e[0;37m'

# The workspace root: the folder colcon builds in, where build/ install/ log/ appear.
# On the Pi it is the checkout itself (~/ROCKO-env: no src/ folder, no ~/ros2_ws), which is the
# default. In the Docker dev container the checkout sits INSIDE the workspace, at
# /ros2_ws/src/ROCKO-env, so docker/compose.yml sets ROS2_WS=/ros2_ws there.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS2_WS:-$here}"

echo -e "${yellow}Building ROS packages in ${WS}..."
echo -e "${white}"
# Stop here if the workspace is missing: without this, every later command
# would run in whatever directory the script happened to be launched from.
cd "$WS" || { echo "ERROR: workspace '$WS' not found. Set ROS2_WS to your workspace root." >&2; exit 1; }
# ROCKO_ROS_SETUP overrides ROS's setup file (the host test uses it); the default is the robot's.
source "${ROCKO_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

echo -e "${yellow}Updating package list..."
echo -e "${white}"
sudo apt-get update

echo -e "${yellow}Installing dependencies..."
echo -e "${white}"
# -r keeps going past a dependency rosdep cannot install, so a missing runtime package shows up at
# launch (Tier 2) rather than failing the build here.
# A workspace with a src/ folder (the container) is searched there; a checkout that IS the workspace
# (the Pi; this repo has no top-level src/) is searched in its two package folders, not the whole
# tree: build/ and install/ hold copies of every package.xml.
if [ -d src ]; then package_paths=(src); else package_paths=(rocko_env rocko_interfaces); fi
rosdep install --from-paths "${package_paths[@]}" --ignore-src -r -y

# 3. On the Pi only: install WiringPi first, or step 4 stops with "WiringPi not found"
#    https://github.com/WiringPi/WiringPi

echo -e "${yellow}Building ROS packages..."
echo -e "${white}"
# Off the Pi there is no WiringPi, so the build must be told to leave out the motor plugin.
# docker/compose.yml sets ROCKO_SKIP_WIRINGPI=ON in the container; on the Pi it is unset and
# the build requires WiringPi. The value is always passed, even when OFF: CMake remembers the
# last -D between builds, so passing nothing could leave an old ON in place.
colcon build --symlink-install --cmake-args -DROCKO_SKIP_WIRINGPI="${ROCKO_SKIP_WIRINGPI:-OFF}" \
  || { echo "ERROR: colcon build failed (see above)." >&2; exit 1; }   # a failed build must not end in "Done!" and exit 0

# clear color coding with \e[0m
echo -e "${yellow}Done!\e[0m"
