FROM ros:humble-ros-base

RUN apt update
RUN apt update && apt full-upgrade -y
RUN apt install -y git python3-pip
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt install -y ros-${ROS_DISTRO}-navigation2
RUN apt install -y ros-${ROS_DISTRO}-nav2*
RUN apt install -y vim

# Install groot
RUN sudo apt install -y git qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev cmake
RUN git clone https://github.com/BehaviorTree/Groot.git
RUN cd /Groot && git submodule update --init --recursive && mkdir build && cd build && cmake .. && make

# Camera and Opencv
RUN sudo apt install -y libopencv-dev python3-opencv python3-numpy
RUN sudo apt install -y ros-${ROS_DISTRO}-vision-opencv
RUN pip3 install imutils

#RUN git clone https://github.com/VicenteMoraes/skill_library
COPY skill_library /skill_library
RUN pip3 install -e /skill_library


WORKDIR skill_library
