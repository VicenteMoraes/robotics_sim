import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ


def test_rviz():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=True, auto_remove=True, network=network)
    robot = Turtlebot3(client, auto_remove=False, network=network)
    rviz = RVIZ(client, auto_remove=False, network=network)

    network.build()
    sim.build()
    robot.build()
    rviz.build()

    sim.run(network_mode=False)
    robot.run()
    rviz.run()
