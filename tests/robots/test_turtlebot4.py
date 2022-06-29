import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot4 import Turtlebot4
from plugins.networks.ros2_network import ROS2Network


def test_turtlebot4():
    client = docker.from_env()
    network = ROS2Network(docker_client=client)
    sim = Gazebo(client, headless=False, auto_remove=False, network=network)
    robot = Turtlebot4(docker_client=client, tag="robot1", network=network)

    network.build()
    sim.build()
    robot.build()

    sim.run()
    robot.run()