import docker
from plugins.networks.network import Network


class ROS2Network(Network):
    def __init__(self, docker_client: docker.DockerClient, priority: int = 1, name: str = "ros2_network",
                 driver: str = "bridge", check_duplicate: bool = True, internal: bool = False, **options):
        super(ROS2Network, self).__init__(docker_client=docker_client, priority=priority, name=name, driver=driver,
                                          check_duplicate=check_duplicate, internal=internal, **options)
