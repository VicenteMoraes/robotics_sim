FROM ros:humble-ros-base

RUN apt update ** apt full-upgrade -y
RUN apt install -y ros-${ROS_DISTRO}-py-trees ros-${ROS_DISTRO}-py-trees-tutorials
RUN apt install -y git

RUN git clone https://github.com/Gastd/py_trees_ros_behaviors/