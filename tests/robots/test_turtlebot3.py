import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.networks.ros2_network import ROS2Network


def test_turtlebot3():
    client = docker.from_env()
    network = ROS2Network(docker_client=client, name="ros2")
    sim = Gazebo(docker_client=client, headless=True, auto_remove=False, network=network)
    robot = Turtlebot3(docker_client=client, tag="robot1", network=network)

    network.build()
    sim.build()
    robot.build()

    sim.run(network_mode=False)
    robot.run()
    assert True