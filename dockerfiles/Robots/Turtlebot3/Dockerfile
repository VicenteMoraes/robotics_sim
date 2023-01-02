FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y
RUN apt update

RUN apt install -y ros-${ROS_DISTRO}-turtlebot3*
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
