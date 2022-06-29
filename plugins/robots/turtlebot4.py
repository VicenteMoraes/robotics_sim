from plugins.robots.robot import Robot
from core.components import ProjectPath
import docker

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot4")


class Turtlebot4(Robot):
    def __init__(self, path: str = DEFAULT_PATH, tag: str = "turtlebot4",
                 command="ros2 launch --debug /workdir/launch/robot_bringup.py", container_name: str = "turtlebot4",
                 *args, **kwargs):
        super(Turtlebot4, self).__init__(path=path, container_name=container_name, tag=tag, command=command,
                                         *args, **kwargs)

        if path == DEFAULT_PATH:
            self.add_mount(f"{DEFAULT_PATH}/launch:/workdir/launch")