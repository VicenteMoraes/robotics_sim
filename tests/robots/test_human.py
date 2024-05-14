import docker
from modules.simulators.gazebo import Gazebo
from modules.robots.human import Human
from modules.networks.ros2_network import ROS2Network


def test_human():
    return
    client = docker.from_env()
    network = ROS2Network(docker_client=client, name="human_test")
    sim = Gazebo(docker_client=client, headless=False, auto_remove=True, network=network)
    sim.add_logger(write_to_file=True, filename="sim.log")
    human = Human(docker_client=client, auto_remove=True, network=network)
    human.add_logger(write_to_file=True, filename="human.log")

    network.build()
    sim.build()
    human.build()

    sim.run(network_mode=False)
    human.run()
