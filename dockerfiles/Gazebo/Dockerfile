FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y

RUN apt install -y xauth
RUN apt install -y ros-${ROS_DISTRO}-gazebo-*
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt install -y wget
RUN wget -nH -r http://models.gazebosim.org/ -P ~/.gazebo/models
