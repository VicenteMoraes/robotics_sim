from plugins.robots.robot import Robot
from core.components import ProjectPath
from core.pose import Pose
from docker import DockerClient

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3")


class Turtlebot3(Robot):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, command=None, tag: str = "turtlebot3",
                 robot_name: str = "turtlebot", robot_namespace: str = "turtlebot", initial_pose: Pose = None,
                 sdf_file: str = '/workdir/launch/waffle.model', *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command,
                                         robot_name=robot_name, robot_namespace=robot_namespace,
                                         initial_pose=initial_pose, *args, **kwargs)
        if command is None:
            self.command = f"ros2 launch /workdir/launch/robot_bringup.py"

        if path == DEFAULT_PATH:
            self.add_mount(source=f"{DEFAULT_PATH}/launch", target="/workdir/launch")

        self.env['ROBOT_SDF'] = sdf_file
