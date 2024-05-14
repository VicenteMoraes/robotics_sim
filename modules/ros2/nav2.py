from docker import DockerClient
from modules.base_module import DockerModule
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/Nav2")


class NAV2(DockerModule):
    def __init__(self, docker_client: DockerClient, robot_name: str, robot_namespace: str, path: str = DEFAULT_PATH,
                 command: str = "", tag: str = "nav2", *args, **kwargs):
        super(NAV2, self).__init__(docker_client=docker_client, path=path, command=command, tag=tag, *args, **kwargs)
        self.robot_name = robot_name
        self.robot_namespace = robot_namespace
