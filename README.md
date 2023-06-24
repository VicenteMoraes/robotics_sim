# Generic Robotics Simulator with Docker

Library for implementing Distributed Robotics experiment trials with Docker in Python.

## Dependencies

- Python >= 3.10
- Docker
- Install all python dependencies listed on requirements.txt
- [OPTIONAL] nvidia-driver and nvidia-docker-runtime (required to run RVIZ and Gazebo Ignition GUI)

## Installation

```bash
git clone https://github.com/VicenteMoraes/robotics_sim 
pip3 install -e robotics_sim
```

## Usage

This library provides an accessible interface for integrating different containers as plugins in a distributed system using docker.

Integrate different aspects of your ros 2 simulation, such as RVIZ, Gazebo, Nav2 and each robot as a separate container.
    Or organise your simulations into trials to be run distributively.

## Info

In order to run simulations with a GUI you need to run the following command in your terminal:

```bash
xhost +local:docker
```

Also make sure your user is part of the docker group:

```bash
sudo groupadd docker
sudo usermod -aG docker $USER
```

## Examples

- Run a gazebo simulation with a turtlebot3

```python
import docker
from robotics_sim.plugins.simulators.gazebo import Gazebo
from robotics_sim.plugins.robots.turtlebot3 import Turtlebot3
from robotics_sim.plugins.networks.ros2_network import ROS2Network


client = docker.from_env()
network = ROS2Network(docker_client=client, name="ros2")
sim = Gazebo(docker_client=client, headless=True, auto_remove=True, network=network)
robot = Turtlebot3(docker_client=client, tag="robot1", auto_remove=True, network=network)

network.build()
sim.build()
robot.build()

sim.run()
robot.run()
```

- Run one of the RMF_Gazebo demos

```python
import docker
from robotics_sim.plugins.simulators.rmf_gazebo_simulator import RMFGazebo

client = docker.from_env()
sim = RMFGazebo(client, headless=True)
sim.build()
sim.run()
```

- Run a complete turtlebot3 simulation with nav2 and gazebo with logging.

```python
import docker
from robotics_sim.plugins.simulators.gazebo import Gazebo
from robotics_sim.plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from robotics_sim.plugins.networks.ros2_network import ROS2Network
from robotics_sim.plugins.loggers.docker_logger import DockerLogger
from robotics_sim.core import pose


client = docker.from_env()
network = ROS2Network(client, name="ros2")
sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world")
sim.add_logger(write_to_file=True, filename='sim.log')

ps = pose.Pose()
ps.position.x = -2
ps.position.y = -0.5
ps.position.z = 0.1
robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network,
                           initial_pose=ps, use_rviz=True)
robot.add_logger(write_to_file=True, filename='robot.log')
sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")

ps = pose.Pose()
ps.position.x = 2
ps.position.y = -1
ps.position.z = 0.1
robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2",
                            auto_remove=True, network=network, initial_pose=ps, use_rviz=True)
robot2.add_logger(write_to_file=True, filename='robot2.log')

network.build()
sim.build()
robot.build()
robot2.build()

sim.run()
robot.run()
robot2.run()
```

- Run turtlebot4 simulation on a separate host with ssh.
```python

import docker
from robotics_sim.plugins.simulators.gazebo_ignition import GazeboIgnition
from robotics_sim.plugins.robots.turtlebot4 import Turtlebot4
from robotics_sim.plugins.networks.ros2_network import ROS2Network


client = docker.DockerClient(base_url="ssh://username@your_host")
network = ROS2Network(client, name="ros2")
sim = GazeboIgnition(docker_client=client, headless=False, network=network, path_to_world="/workdir/map/cicmap.sdf")
sim.add_logger(write_to_file=True, filename="ignition.log")
robot = Turtlebot4(docker_client=client, tag="tb4", auto_remove=True, network=network, use_rviz=True, use_nav2=False,
                   use_slam=False, use_localization=False)
robot.add_logger(write_to_file=True, filename="tb4.log")

network.build()
sim.build()
robot.build()

sim.run()
robot.run()
```

You can also find more examples in our tests directory.

## Images

### Simulation Example
![hospital map](imgs/hospital_map.png) 

### Trial structure
![plugins](imgs/plugin.drawio.png)

![trials](imgs/sim.drawio.png)
