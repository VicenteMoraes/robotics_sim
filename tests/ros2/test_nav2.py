import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ
from plugins.ros2.nav2 import NAV2
from plugins.loggers.docker_logger import DockerLogger


def test_nav2():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=True, auto_remove=True, network=network)
    sim.add(DockerLogger(target='', write_to_file=True, filename='sim.log'))
    robot = Turtlebot3(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network)
    robot.add(DockerLogger(target='', write_to_file=True, filename='robot.log'))
    rviz = RVIZ(client, auto_remove=True, network=network)
    rviz.add(DockerLogger(target='', write_to_file=True, filename='rviz.log'))
    nav2 = NAV2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=False, network=network)

    network.build()
    sim.build()
    robot.build()
    rviz.build()
    nav2.build()

    sim.run(network_mode=False)
    robot.run()
    rviz.run()
    nav2.run()
