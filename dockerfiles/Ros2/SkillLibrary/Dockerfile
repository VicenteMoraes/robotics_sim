FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y
RUN apt install -y git python3-pip
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
RUN apt install -y ros-${ROS_DISTRO}-nav2-rviz-plugins

RUN git clone https://github.com/VicenteMoraes/skill_library
RUN pip3 install -r /skill_library/requirements.txt
