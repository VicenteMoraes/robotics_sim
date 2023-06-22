from docker import DockerClient
from core.pose import Pose, quaternion_from_euler
from trials.trial import Trial
from plugins.ros2.rviz import RVIZ
from plugins.simulators.gazebo import Gazebo
from plugins.networks.ros2_network import ROS2Network
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.ros2.skill_library import SkillLibrary
from plugins.robots.human import Human
from plugins.ros2.roslogger import ROSLogger


class HMRSTrial(Trial):
    def __init__(self, docker_client: DockerClient, config, trial_id: int, trial_code: str, headless: bool = True,
                 network_name: str = "", path_to_world: str = "", use_rviz: bool = False,  sim_timeout: float = 15*60,
                 dir: str = "", target: str = 'WARN', *logger_args, **logger_kwargs):
        super(HMRSTrial, self).__init__(trial_id=trial_id)
        self.docker_client = docker_client
        self.headless = headless
        self.trial_id = f"{trial_id}_{trial_code}"
        self.network_name = network_name if network_name else f"trial_{self.trial_id}_net"
        self.config = config
        self.use_rviz = use_rviz
        self.sim_timeout = sim_timeout
        self.dir = dir

        self.network = ROS2Network(self.docker_client, name=self.network_name)
        self.sim = Gazebo(self.docker_client, headless=headless, network=self.network, path_to_world=path_to_world)
        self.skills = {}

        self.logger = ROSLogger(self.docker_client, self.network, trial_id=trial_id, filename=f"{dir}/{self.trial_id}.log",
                                timeout=self.sim_timeout, target=target, *logger_args, **logger_kwargs)

        self.add_plugins(self.sim, self.network, self.logger)

    @staticmethod
    def get_pose_from_loc(loc):
        poses = {
            "IC Corridor": [-37, 15],
            "IC Room 1": [-39.44, 33.98, 0.00],
            "IC Room 2": [-32.88, 33.95, 3.14],
            "IC Room 3": [-40.23, 25.37, 0.00],
            "IC Room 4": [-33.90, 18.93, 3.14],
            "IC Room 5": [-38.00, 21.50, 0.00],
            "IC Room 6": [-38.00, 10.00, 0.00],
            "PC Corridor": [-19, 16],
            "PC Room 1": [-28.50, 18.00,-1.57],
            "PC Room 2": [-27.23, 18.00,-1.57],
            "PC Room 3": [-21.00, 18.00,-1.57],
            "PC Room 4": [-19.00, 18.00,-1.57],
            "PC Room 5": [-13.50, 18.00,-1.57],
            "PC Room 6": [-11.50, 18,-1.57],
            "PC Room 7": [-4, 18,-1.57],
            "PC Room 8": [-27.23, 13.00, 1.57],
            "PC Room 9": [-26.00, 13.00, 1.57],
            "PC Room 10": [-18.00, 13.00, 1.57],
            "Reception": [-1, 20],
            "Pharmacy Corridor": [0, 8],
            "Pharmacy": [-2, 2.6],
        }
        return poses[loc]

    @staticmethod
    def pose_from_config(config):
        pose = Pose()
        pose.position.x = config['position'][0]
        pose.position.y = config['position'][1]

        quat = quaternion_from_euler(0, 0, config['position'][2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def setup_robots(self, *robot_args, **robot_kwargs):
        for config in self.config['robots']:
            try:
                pose = self.pose_from_config(config)
            except KeyError:
                pose = self.get_pose_from_loc(config['location'])

            name = f"robot{config['id']}"
            if config['local_plan']:
                robot = Turtlebot3withNav2(self.docker_client, robot_name=name, config=config, use_rviz=self.use_rviz,
                                           robot_namespace=name, initial_pose=pose, network=self.network,
                                           *robot_args, **robot_kwargs)
                self.sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")

                skill_library = SkillLibrary(self.docker_client, config=config, network=self.network, robot_name=name,
                                             robot_namespace=name, initial_pose=pose)
                self.skills[name] = skill_library
                self.add_plugins(skill_library)
            else:
                robot = Turtlebot3(self.docker_client, robot_name=name, container_name=f"{name}_container", config=config,
                                   robot_namespace=name, initial_pose=pose, network=self.network)
            self.add_plugins(robot)

    def setup_nurse(self, *robot_args, **robot_kwargs):
        for config in self.config['nurses']:
            pose = self.pose_from_config(config)
            human = Human(self.docker_client, robot_name="nurse", config=config, initial_pose=pose, network=self.network,
                          *robot_args, **robot_kwargs)
            self.add_plugins(human)

    def run(self):
        super(HMRSTrial, self).run()
        self.logger.start_logger()

