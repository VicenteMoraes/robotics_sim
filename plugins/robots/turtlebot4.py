from plugins.robots.robot import Robot
from core.components import ProjectPath
from core.pose import Pose
from docker import DockerClient

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot4")


class Turtlebot4(Robot):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, tag: str = "turtlebot4",
                 command: str = "ros2 launch --debug /workdir/launch/tb4_bringup.launch.py slam:=true",
                 robot_name: str = "turtlebot4", robot_namespace: str = "turtlebot4", initial_pose: Pose = None,
                 sim_hostname: str = 'gz_sim', use_rviz: bool = False, use_nav2: bool = False, use_slam: bool = False,
                 spawn_dock: bool = False, dock_pose: Pose = None, robot_model: str = "lite", map_dir: str = "",
                 map_yaml: str = '/workdir/map/map.yaml', *args, **kwargs):
        super(Turtlebot4, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command,
                                         robot_name=robot_name, robot_namespace=robot_namespace,
                                         initial_pose=initial_pose, *args, **kwargs)

        self.env['IGN_PARTITION'] = f'{sim_hostname}:root'
        self.use_rviz = use_rviz
        self.env['USE_RVIZ'] = use_rviz
        self.env['USE_NAV2'] = use_nav2
        self.env['USE_SLAM'] = use_slam
        self.robot_model = robot_model
        self.env['ROBOT_MODEL'] = robot_model
        self.env['MAP_YAML'] = map_yaml

        self.env['SPAWN_DOCK'] = spawn_dock
        try:
            self.env['DOCK_POSE'] = dock_pose.pose_to_json()
        except AttributeError:
            self.env['DOCK_POSE'] = Pose().pose_to_json()

        if self.use_rviz:
            self.env['NVIDIA_VISIBLE_DEVICES'] = 'all'
            self.env['NVIDIA_DRIVER_CAPABILITIES'] = 'all'
            self.env['MESA_GL_VERSION_OVERRIDE'] = '4.2'
            self.load_gui()

        self.add_mount(source=f"{DEFAULT_PATH}/launch", target="/workdir/launch")
        if map_dir:
            self.add_mount(source=map_dir, target="/workdir/map")

    def run(self,  **run_kwargs):
        if self.use_rviz:
            run_kwargs['runtime'] = 'nvidia'
        super(Turtlebot4, self).run(**run_kwargs)
