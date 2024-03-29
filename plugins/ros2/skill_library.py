from plugins.plugin import Plugin
from core.components import ProjectPath
from docker import DockerClient
from plugins.networks.ros2_network import ROS2Network
from core.pose import Pose


DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/SkillLibrary")


class SkillLibrary(Plugin):
    def __init__(self, docker_client: DockerClient, config: str, initial_pose: Pose, network: ROS2Network,
                 path: str = DEFAULT_PATH, robot_name: str = 'robot', robot_namespace: str = 'robot',
                 tag: str = "skill_library", command: str = "python3 launch/run_from_config.py", *args, **kwargs):
        super(SkillLibrary, self).__init__(docker_client=docker_client, network=network, path=path, tag=tag,
                                           command=command, *args, **kwargs)

        self.env['CONFIG'] = str(config).replace('\'', '\"')
        self.env['ROBOT_NAME'] = robot_name
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.env['ROBOT_POSE'] = initial_pose.pose_to_json()
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
