from plugins.robots.turtlebot3 import Turtlebot3
from docker import DockerClient
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3_nav2")


class Turtlebot3withNav2(Turtlebot3):
    def __init__(self, docker_client: DockerClient, docker_path: str = DEFAULT_PATH, tag: str = "turtlebot3_nav2",
                 command: str = None, launch_path: str = f"{DEFAULT_PATH}/launch", param_path: str = f"{DEFAULT_PATH}/param",
                 models_path: str = f"{DEFAULT_PATH}/models", map_yaml: str = '', params_yaml: str = '',
                 turtlebot3_model: str = 'burger', lds_model: str = 'LDS-01', robot_name: str = "turtlebot",
                 robot_namespace: str = "turtlebot", use_rviz: bool = False, use_slam: bool = False,
                 use_pose_logger: bool = True, use_battery: bool = True,
                 *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=docker_path, tag=tag, command=command,
                                         robot_name=robot_name, robot_namespace=robot_namespace, *args, **kwargs)

        self.env['TURTLEBOT3_MODEL'] = turtlebot3_model
        self.env['LDS_MODEL'] = lds_model
        self.env['MAP_YAML'] = map_yaml
        self.env['PARAMS_YAML'] = params_yaml
        self.use_rviz = use_rviz
        self.env['USE_RVIZ'] = str(use_rviz)

        if self.use_rviz:
            self.load_gui()

        if command is None:
            if use_slam:
                self.command = f"""bash -c "ros2 launch /workdir/launch/robot_bringup.py \
                && (ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py \
                & ros2 launch nav2_bringup navigation_launch.py \
                & ros2 launch slam_toolbox online_async_launch.py) """
                #&& (ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py \
                #& ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True) """
            else:
                self.command = f"""bash -c "ros2 launch -d /workdir/launch/nav2_bringup.launch.py """

            if use_battery:
                self.command += "& python3 /workdir/launch/battery.py "
            if use_pose_logger:
                self.command += "& python3 /workdir/launch/robot_logger.py "

            self.command += '"'

        self.add_mount(source=launch_path, target="/workdir/launch")
        self.add_mount(source=param_path, target="/workdir/param")
        self.add_mount(source=models_path, target="/workdir/models")

    def run(self, **run_kwargs):
        if self.use_rviz:
            run_kwargs['runtime'] = 'nvidia'
        super(Turtlebot3withNav2, self).run(**run_kwargs)