import docker
from core.pose import Pose, quaternion_from_euler
from trials.trial import Trial
from plugins.ros2.rviz import RVIZ
from plugins.simulators.gazebo import Gazebo
from plugins.networks.ros2_network import ROS2Network
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.robots.human import Human
from plugins.ros2.roslogger import ROSLogger


class HMRSTrial(Trial):
    def __init__(self, config, trial_id: int, trial_code: str, headless: bool = True, network_name: str = "",
                 path_to_world: str = "", use_rviz: bool = True, logger_args: list = None, logger_kwargs: dict = None):
        super(HMRSTrial, self).__init__(trial_id=trial_id)
        self.docker_client = docker.from_env()
        self.headless = headless
        self.trial_id = f"{trial_id}_{trial_code}"
        self.network_name = network_name if network_name else f"trial_{trial_id}_net"
        self.config = config
        self.use_rviz = use_rviz
        logger_args = logger_args if logger_args is not None else []
        logger_kwargs = logger_kwargs if logger_kwargs is not None else {}

        self.network = ROS2Network(self.docker_client, name=self.network_name)
        self.sim = Gazebo(self.docker_client, headless=headless, network=self.network, path_to_world=path_to_world)
        self.sim.add_logger(write_to_file=True, filename="sim.log", *logger_args, **logger_kwargs)

        if use_rviz:
            self.rviz = RVIZ(self.docker_client, network=self.network)
            self.add_plugins(self.rviz)

        self.logger = ROSLogger(self.docker_client, self.network, trial_id=trial_id, filename=f"{trial_id}.log")

        self.add_plugins(self.sim, self.network, self.logger)

    def pose_from_config(self, config):
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
            pose = self.pose_from_config(config)
            name = f"robot{config['id']}"
            if config['local_plan']:
                robot = Turtlebot3withNav2(self.docker_client, robot_name=name, config=config,
                                           robot_namespace=name, initial_pose=pose, network=self.network,
                                           *robot_args, **robot_kwargs)
                robot.add_logger(write_to_file=True, filename="robot.log")
                self.sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
            else:
                robot = Turtlebot3(self.docker_client, robot_name=name, container_name=f"{name}_container", config=config,
                                   robot_namespace=name, initial_pose=pose, network=self.network)
            self.add_plugins(robot)

    def setup_nurse(self, *robot_args , **robot_kwargs):
        for config in self.config['nurses']:
            pose = self.pose_from_config(config)
            human = Human(self.docker_client, robot_name="nurse", config=config, initial_pose=pose, network=self.network,
                          *robot_args, **robot_kwargs)
            self.add_plugins(human)
