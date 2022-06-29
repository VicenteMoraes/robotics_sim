from plugins.robots.robot import Robot
from core.components import ProjectPath
import docker

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot4")


class Turtlebot4(Robot):
    def __init__(self, docker_client, path: str = DEFAULT_PATH, priority: int = 5, tag: str = "turtlebot4", network=None,
                 command="ros2 launch --debug /workdir/launch/robot_bringup.py",
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, container_name: str = "turtlebot4"):
        super(Turtlebot4, self).__init__(docker_client=docker_client, path=path, priority=priority, network=network,
                                         container_name=container_name, tag=tag, dockerfile=dockerfile,
                                         auto_remove=auto_remove, command=command)

        if path == DEFAULT_PATH:
            self.add_mount(f"{DEFAULT_PATH}/launch:/workdir/launch")