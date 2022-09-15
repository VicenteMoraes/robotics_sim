from plugins.robots.robot import Robot
from core.components import ProjectPath
from docker import DockerClient

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3")


class Turtlebot3(Robot):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, command=None, tag: str = "turtlebot3",
                 robot_name: str = "turtlebot", robot_namespace: str = "turtlebot", *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command,
                                         robot_name=robot_name, *args, **kwargs)
        if command is None:
            self.command = f"python3 /workdir/launch/robot_bringup.py {robot_name} {robot_namespace} {str(self.initial_pose)} "

        if path == DEFAULT_PATH:
            self.add_mount(source=f"{DEFAULT_PATH}/launch", target="/workdir/launch")

