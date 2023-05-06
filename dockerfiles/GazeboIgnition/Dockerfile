FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y

RUN apt install -y xauth
RUN apt install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update
RUN apt install -y gz-garden
RUN apt install -y ros-${ROS_DISTRO}-ros-ign-bridge
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
