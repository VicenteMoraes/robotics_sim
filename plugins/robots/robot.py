from plugins.plugin import DockerPlugin
import docker


class Robot(DockerPlugin):
    def __init__(self, docker_client, path: str, command: str = "", priority: int = 5, tag: str = "", network=None,
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, container_name: str = "robot"):
        super(Robot, self).__init__(docker_client=docker_client, path=path, priority=priority, network=network,
                                    container_name=container_name, tag=tag, dockerfile=dockerfile,
                                    auto_remove=auto_remove, command=command)
