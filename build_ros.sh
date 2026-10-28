#!/usr/bin/env bash
# Builds the ROS packages
# Intended to be ran inside the Docker container or on the robot
yellow='\e[0;33m'
white='\e[0;37m'

# The workspace root (the folder that contains src/). On the Pi this is ~/ros2_ws.
# In the Docker dev container it is /ros2_ws, so docker/compose.yml sets ROS2_WS there.
WS="${ROS2_WS:-$HOME/ros2_ws}"

echo -e "${yellow}Building ROS packages in ${WS}..."
echo -e "${white}"
# Stop here if the workspace is missing: without this, every later command
# would run in whatever directory the script happened to be launched from.
cd "$WS" || { echo "ERROR: workspace '$WS' not found. Set ROS2_WS to your workspace root." >&2; exit 1; }
source /opt/ros/jazzy/setup.bash

echo -e "${yellow}Updating package list..."
echo -e "${white}"
sudo apt-get update

echo -e "${yellow}Installing dependencies..."
echo -e "${white}"
# -r keeps going past a dependency rosdep cannot install, so a missing runtime package shows up at
# launch (Tier 2) rather than failing the build here.
rosdep install --from-paths src --ignore-src -r -y

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
