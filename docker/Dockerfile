# Dev image for ROCKO-env. One image, used from both Mac and Windows hosts;
# the OS-specific bits (GUI display forwarding, networking) live in the
# compose override files, not here.
#
# This image does NOT bake in ROCKO's ROS package dependencies (rosdep does
# that at container start from the mounted package.xml files, same as the
# non-Docker build in the top-level README) and it cannot build the WiringPi
# hardware plugin off a Raspberry Pi - see the README section above.
FROM ros:jazzy

ARG USERNAME=rocko
ARG USER_UID=1001
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    iputils-ping \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
ENV SHELL=/bin/bash

USER $USERNAME
RUN rosdep update

WORKDIR /ros2_ws
CMD ["/bin/bash"]
