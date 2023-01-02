from plugins.plugin import DockerPlugin
from docker import DockerClient
from core.pose import Pose


class Robot(DockerPlugin):
    def __init__(self, docker_client: DockerClient, path: str, command: str, initial_pose: Pose,
                 tag: str = "robot", robot_name: str = "robot", robot_namespace: str = "robot",
                 config: str = '', *args, **kwargs):
        super(Robot, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command, *args, **kwargs)
        self.initial_pose = initial_pose if initial_pose is not None else Pose()
        self.robot_name = robot_name
        self.env['ROBOT_NAME'] = robot_name
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.config = config
        self.env['CONFIG'] = str(config).replace('\'', '\"')
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.env['ROBOT_POSE'] = self.initial_pose.pose_to_json()
