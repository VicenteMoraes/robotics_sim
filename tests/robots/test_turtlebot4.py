import docker
from plugins.simulators.gazebo_ignition import GazeboIgnition
from plugins.robots.turtlebot4 import Turtlebot4
from plugins.networks.ros2_network import ROS2Network
from core.components import ProjectPath


def test_turtlebot4():
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = GazeboIgnition(docker_client=client, headless=False, network=network)
    sim.add_logger(write_to_file=True, filename="ignition.log")
    robot = Turtlebot4(docker_client=client, tag="tb4", auto_remove=True, network=network, use_rviz=True, use_nav2=True,
                       map_dir=str(ProjectPath/"tests/robots/cicmap"))
    robot.add_logger(write_to_file=True, filename="tb4.log")

    network.build()
    sim.build()
    robot.build()

    sim.run()
    robot.run()
