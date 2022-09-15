FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
