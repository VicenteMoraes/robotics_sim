from plugins.plugin import DockerPlugin
from docker import DockerClient
from core.pose import Pose, euler_from_quaternion


class Robot(DockerPlugin):
    def __init__(self, docker_client: DockerClient, path: str, command: str, tag: str = "robot", robot_name: str = "robot",
                 initial_pose: Pose = None, config: str = '', *args, **kwargs):
        super(Robot, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command, *args, **kwargs)
        self.initial_pose = initial_pose if initial_pose is not None else Pose()
        self.robot_name = robot_name
        self.env['ROBOT_NAME'] = robot_name
        self.config = config
        self.env['CONFIG'] = str(config).replace('\'', '\"')
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
