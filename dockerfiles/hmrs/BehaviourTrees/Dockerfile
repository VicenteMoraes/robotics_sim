FROM ros:foxy-ros-base

RUN apt update && apt full-upgrade -y
RUN apt install -y ros-${ROS_DISTRO}-py-trees ros-${ROS_DISTRO}-py-trees-ros ros-${ROS_DISTRO}-py-trees-ros-tutorials
RUN apt install -y git
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

WORKDIR /ros_ws

RUN mkdir -p /ros_ws/src
COPY py_trees_ros_behaviors /ros_ws/src/py_trees_ros_behaviors
RUN /bin/bash -c "source /ros_entrypoint.sh && cd /ros_ws/ && colcon build"

#RUN git clone https://github.com/VicenteMoraes/py_trees_ros_behaviors/
#COPY py_trees_ros_behaviors py_trees_ros_behaviors
#RUN bash -c "cd py_trees_ros_behaviors/bt_skills && git clone https://github.com/lesunb/hmrs_mission_control"
#WORKDIR py_trees_ros_behaviors
