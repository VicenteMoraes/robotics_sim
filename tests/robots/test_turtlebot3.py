import docker
from modules.simulators.gazebo import Gazebo
from modules.robots.turtlebot3 import Turtlebot3
from modules.networks.ros2_network import ROS2Network


def test_turtlebot3():
    return
    client = docker.from_env()
    network = ROS2Network(docker_client=client, name="ros2")
    sim = Gazebo(docker_client=client, headless=False, auto_remove=True, network=network)
    robot = Turtlebot3(docker_client=client, tag="robot1", auto_remove=True, network=network)

    network.build()
    sim.build()
    robot.build()

    sim.run(network_mode=False)
    robot.run()