# Builds the ROS packages
# Intended to be ran inside the Docker container or on the robot
yellow='\e[0;33m'
white='\e[0;37m'

echo -e "${yellow}Building ROS packages..."
echo -e "${white}"
cd ~/ros2_ws
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
