from plugins.robots.robot import Robot
from core.components import ProjectPath
import docker

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3")


class Turtlebot3(Robot):
    def __init__(self, path: str = DEFAULT_PATH, command=None, tag: str = "turtlebot3", robot_name: str = "turtlebot",
                 robot_namespace: str = "turtlebot", container_name: str = "turtlebot3", initial_pose=None,
                 *args, **kwargs):
        if initial_pose is None:
            self.initial_pose = [0, 0, 0.1]
        if command is None:
            self.command = f"ros2 run --debug /workdir/launch/robot_bringup.py {robot_name} {robot_namespace} {' '.join(initial_pose)}"
        super(Turtlebot3, self).__init__(path=path, container_name=container_name, tag=tag, command=command,
                                         robot_name=robot_name, *args, **kwargs)

        if path == DEFAULT_PATH:
            self.add_mount(f"{DEFAULT_PATH}/launch:/workdir/launch")