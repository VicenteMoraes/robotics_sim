FROM ros:humble-ros-base

RUN apt update && apt full-upgrade -y

RUN apt install -y git wget
RUN apt update
RUN apt install -y ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup
RUN apt install -y ros-${ROS_DISTRO}-nav2-rviz-plugins
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Building Gazebo Ignition Garden
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
#RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt update && apt full-upgrade -y
RUN echo "deb http://packages.ros.org/ros2-testing/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list
RUN apt update
RUN apt install -y gz-fortress ros-${ROS_DISTRO}-ros-ign

# Building envs
ENV IGNITION_VERSION=fortress
ENV GZ_VERSION=fortress

# Building Create 3 sim and tb4 simulator
RUN apt install -y ros-${ROS_DISTRO}-turtlebot4-desktop
RUN apt install -y ros-${ROS_DISTRO}-turtlebot4-simulator

#UN mkdir -p /tb4_ws/src
#ORKDIR /tb4_ws/src
#UN git clone https://github.com/turtlebot/turtlebot4_simulator.git -b ${ROS_DISTRO}
#UN bash -c "source /opt/ros/humble/setup.bash && rosdep install --from-path . -yi"

#ORKDIR /tb4_ws
#UN bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Reset workdir
WORKDIR /

# Building tb4
#RUN mkdir -p /turtlebot4_ws/src
#WORKDIR /turtlebot4_ws/src
#RUN apt update
#RUN apt install -y ros-${ROS_DISTRO}-turtlebot4-desktop ros-${ROS_DISTRO}-turtlebot4-msgs
#RUN git clone https://github.com/turtlebot/turtlebot4_simulator.git -b ${ROS_DISTRO}
#WORKDIR /turtlebot4_ws
#RUN colcon build --symlink-install
