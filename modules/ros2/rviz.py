from modules.base_module import DockerModule
from core.components import ProjectPath
from docker import DockerClient

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/Rviz")


class RVIZ(DockerModule):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, command: str = "",
                 tag: str = "rviz", config_file: str = "", *args, **kwargs):
        super(RVIZ, self).__init__(docker_client=docker_client, path=path, command=command, tag=tag, *args, **kwargs)
        self.command = "ros2 run rviz2 rviz2"
        if config_file:
            self.command = f"ros2 run rviz2 rviz2 {config_file}"
        self.load_gui()

    def run(self, **run_kwargs):
        super(RVIZ, self).run(runtime='nvidia', **run_kwargs)
