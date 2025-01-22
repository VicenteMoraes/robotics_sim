from docker import DockerClient
from core.pose import Pose, quaternion_from_euler
from trials.trial import Trial
from modules.ros2.rviz import RVIZ
from modules.simulators.gazebo import Gazebo
from modules.networks.ros2_network import ROS2Network
from modules.robots.turtlebot3_nav2 import Turtlebot3withNav2
from modules.robots.turtlebot3 import Turtlebot3
from modules.ros2.skill_library import SkillLibrary
from modules.robots.human import Human
from modules.ros2.roslogger import ROSLogger
import json


class HMRSTrial(Trial):
    def __init__(self, docker_client: DockerClient, config, ihtn: dict, trial_id: int, headless: bool = True,
                 network_name: str = "", skill_library: str = "", use_rviz: bool = False,  sim_timeout: float = 15*60,
                 dir: str = "", target: str = 'WARN', ssh_host: str = None, ssh_pass: str = '',
                 *logger_args, **logger_kwargs):
        super(HMRSTrial, self).__init__(trial_id=trial_id)
        self.docker_client = docker_client
        self.headless = headless
        self.trial_id = trial_id
        self.network_name = network_name if network_name else f"ros2"
        self.config = config
        self.use_rviz = use_rviz
        self.sim_timeout = sim_timeout
        self.dir = dir
        self.ssh_host = ssh_host
        self.ssh_pass = ssh_pass
        self.ihtn = ihtn

        self.network = ROS2Network(self.docker_client, name=self.network_name)
        self.skill_library = skill_library

        self.sim = None
        self.logger = ROSLogger(self.docker_client, self.network, trial_id=trial_id, filename=f"{dir}/{self.trial_id}.log",
                                timeout=self.sim_timeout, target=target, ssh_host=self.ssh_host, ssh_pass=self.ssh_pass,
                                *logger_args, **logger_kwargs)

        self.add_plugins(self.network, self.logger)

    def setup_simulator(self, simulator: str, path_to_world: str):
        if simulator == "gazebo":
            self.sim = Gazebo(self.docker_client, headless=self.headless, network=self.network,
                              path_to_world=path_to_world,
                              ssh_host=self.ssh_host, ssh_pass=self.ssh_pass)
            self.sim.add_logger(write_to_file=True, filename="sim.log")
        self.add_plugins(self.sim)

    def get_pose_from_loc(self, loc):
        poses = {
            "IC Corridor": [-37.00, 16.00, 0.00],
            "IC Room 1": [-38.00, 10.00, 0.00],
            "IC Room 2": [-38.00, 21.50, 0.00],
            "IC Room 3": [-40.23, 25.37, 0.00],
            "IC Room 4": [-39.44, 33.95, 0.00],
            "IC Room 5": [-32.88, 33.98, 3.14],
            "IC Room 6": [-33.90, 18.93, 3.14],
            "PC Corridor 1": [-27.25, 16.00],
            "PC Corridor 2": [-18.00, 16.00],
            "PC Corridor 3": [-28.50, 16.00],
            "PC Corridor 4": [-27.23, 16.00],
            "PC Corridor 5": [-21.00, 16.00],
            "PC Corridor 6": [-19.00, 16.00],
            "PC Corridor 7": [-13.50, 16.00],
            "PC Corridor 8": [-11.50, 16.00],
            "PC Room 1": [-27.25, 13.00, 1.57],
            "PC Room 2": [-18.00, 13.00, 1.57],
            "PC Room 3": [-28.50, 18.00, -1.57],
            "PC Room 4": [-27.23, 18.00, -1.57],
            "PC Room 5": [-21.00, 18.00, -1.57],
            "PC Room 6": [-19.00, 18.00, -1.57],
            "PC Room 7": [-13.50, 18.00, -1.57],
            "PC Room 8": [-11.50, 18.00, -1.57],
            "Reception": [-1.00, 20.00, 0.00],
            "Pharmacy Corridor": [-25.00, 16.00],
            "Pharmacy": [-26.25, 13.00, 1.57],
        }
        return poses[loc]

    @staticmethod
    def pose_from_list(pose_list):
        pose = Pose()
        pose.position.x = pose_list[0]
        pose.position.y = pose_list[1]

        quat = quaternion_from_euler(0, 0, pose_list[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def setup_robots(self, *robot_args, **robot_kwargs):
        for name, config in self.config['robot_properties'].items():
            config["battery_charge"] = self.config["independent_variables"]["battery_charge"]
            config["nurse_pose"] = self.get_pose_from_loc(self.config["independent_variables"]['location_nurse'])
            config["robot_pose"] = self.get_pose_from_loc(self.config["independent_variables"]['location_robot'])
            config = json.dumps(config)


            if name == "nurse":
                pose = self.get_pose_from_loc(self.config["independent_variables"]['location_nurse'])
                pose = self.pose_from_list(pose)
                robot = Human(self.docker_client, robot_name="nurse", config=config, initial_pose=pose, network=self.network,
                          ssh_host=self.ssh_host, ssh_pass=self.ssh_pass)
            else:
                pose = self.get_pose_from_loc(self.config["independent_variables"]['location_robot'])
                pose = self.pose_from_list(pose)
                robot = Turtlebot3withNav2(self.docker_client, robot_name=name, config=config, use_rviz=self.use_rviz,
                                           robot_namespace=name, initial_pose=pose, network=self.network,
                                           ssh_host=self.ssh_host, ssh_pass=self.ssh_pass, *robot_args, **robot_kwargs)
                robot.add_logger(write_to_file=True, filename="robot.log")
                self.sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
                skill_library = SkillLibrary(self.docker_client, config=config, network=self.network, robot_name=name,
                                         robot_namespace=name, initial_pose=pose, ssh_host=self.ssh_host, ihtn=self.ihtn,
                                         ssh_pass=self.ssh_pass)
                skill_library.add_logger(write_to_file=True, filename="skill.log")
                self.add_plugins(skill_library)
            self.add_plugins(robot)

    def setup_controllers(self):
        pass

    def setup_monitors(self):
        #TODO
        pass

    def setup(self, simulator: str, path_to_world: str, *robot_args, **robot_kwargs):
        self.setup_simulator(simulator, path_to_world)
        self.setup_robots(*robot_args, **robot_kwargs)
        self.setup_controllers()
        self.setup_monitors()

    def run(self):
        super(HMRSTrial, self).run()
        self.logger.start_logger()

