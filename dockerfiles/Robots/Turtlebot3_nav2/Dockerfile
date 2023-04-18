FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y

RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
RUN apt install -y ros-${ROS_DISTRO}-nav2-rviz-plugins
RUN apt install -y ros-${ROS_DISTRO}-turtlebot3*
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt install -y python3-pip
RUN pip install transforms3d


# Install groot
RUN sudo apt install -y git qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev cmake
RUN git clone https://github.com/BehaviorTree/Groot.git
RUN cd /Groot && git submodule update --init --recursive && mkdir build && cd build && cmake .. && make