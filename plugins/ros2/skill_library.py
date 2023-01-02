from plugins.plugin import DockerPlugin
from core.components import ProjectPath
from docker import DockerClient
from plugins.networks.ros2_network import ROS2Network


DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/SkillLibrary")


class SkillLibrary(DockerPlugin):
    def __init__(self, docker_client: DockerClient, config: str, network: ROS2Network, path: str = DEFAULT_PATH,
                 robot_name: str = 'robot', robot_namespace: str = 'robot',
                 tag: str = "skill_library", command: str = "", *args, **kwargs):
        super(SkillLibrary, self).__init__(docker_client=docker_client, network=network, path=path, tag=tag,
                                           command=command, *args, **kwargs)

        self.env['CONFIG'] = str(config).replace('\'', '\"')
        self.env['ROBOT_NAME'] = robot_name
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
