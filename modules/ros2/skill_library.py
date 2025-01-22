from modules.base_module import DockerModule
from core.components import ProjectPath
from docker import DockerClient
from modules.networks.ros2_network import ROS2Network
from core.pose import Pose
import json


DEFAULT_PATH = str(ProjectPath/"dockerfiles/Ros2/SkillLibrary")
DEFAULT_ARM_POSE = [-37.0, 36.00, 0.00]


class SkillLibrary(DockerModule):
    def __init__(self, docker_client: DockerClient, config: str, initial_pose: Pose, network: ROS2Network, ihtn: dict,
                 path: str = DEFAULT_PATH, robot_name: str = 'robot', robot_namespace: str = 'robot', arm_pose: list = None,
                 tag: str = "skill_library", command: str = "python3 /run_from_config.py", *args, **kwargs):
        super(SkillLibrary, self).__init__(docker_client=docker_client, network=network, path=path, tag=tag,
                                           command=command, *args, **kwargs)

        self.env['CONFIG'] = str(config).replace('\'', '\"')
        self.env['ROBOT_NAME'] = robot_name
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.env['ROBOT_POSE'] = initial_pose.pose_to_json()
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.env['NURSE_POSE'] = json.loads(config)['nurse_pose']
        self.env['ARM_POSE'] = json.dumps(DEFAULT_ARM_POSE) if arm_pose is None else arm_pose

        #self.add_mount(source=f"{DEFAULT_PATH}/init", target="/skill_library/startup/run_from_config.py")

        self.task_sequence = []
        self.ihtn = ihtn
        self.task_sequencing()
        self.env['TASKS'] = json.dumps(self.task_sequence)


    def task_sequencing(self, node: str = "0"):
        children = self.ihtn[node]["children"]
        if len(children) == 0:
            self.task_sequence.append(self.ihtn[node]["name"])
        else:
            for child in children:
                self.task_sequencing(child)
