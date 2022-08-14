FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y

RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
RUN apt install -y ros-${ROS_DISTRO}-nav2-rviz-plugins