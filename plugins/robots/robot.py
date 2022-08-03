from plugins.plugin import DockerPlugin
from docker import DockerClient
from core.pose import Pose


class Robot(DockerPlugin):
    def __init__(self, docker_client: DockerClient, path: str, command: str, tag: str = "robot", robot_name: str = "robot",
                 initial_pose: Pose = None, *args, **kwargs):
        super(Robot, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command, *args, **kwargs)
        self.robot_name = robot_name
        self.initial_pose = initial_pose if initial_pose is not None else Pose()
        self.env['ROBOT_NAME'] = robot_name
