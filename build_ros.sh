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
rosdep install --from-paths src --ignore-src -r -y

# 3. On the Pi only: install WiringPi first, or step 4 fails to link
#    https://github.com/WiringPi/WiringPi

echo -e "${yellow}Building ROS packages..."
echo -e "${white}"
colcon build --symlink-install

# clear color coding with \e[0m
echo -e "${yellow}Done!\e[0m"
