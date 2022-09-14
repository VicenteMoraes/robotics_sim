from plugins.robots.turtlebot3 import Turtlebot3
from docker import DockerClient
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3_nav2")


class Turtlebot3withNav2(Turtlebot3):
    def __init__(self, docker_client: DockerClient, docker_path: str = DEFAULT_PATH, tag: str = "turtlebot3_nav2",
                 command: str = None, launch_path: str = f"{DEFAULT_PATH}/launch", param_path: str = f"{DEFAULT_PATH}/param",
                 models_path: str = f"{DEFAULT_PATH}/models", turtlebot3_model: str = 'burger', lds_model: str = 'LDS-01',
                 robot_name: str = "turtlebot", robot_namespace: str = "turtlebot",
                 map_yaml: str = '', *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=docker_path, tag=tag, command=command,
                                         robot_name=robot_name, *args, **kwargs)

        self.env['TURTLEBOT3_MODEL'] = turtlebot3_model
        self.env['LDS_MODEL'] = lds_model
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        self.env['MAP_YAML'] = map_yaml
        if command is None:
            self.command = f"""bash -c "python3 /workdir/launch/robot_bringup.py {robot_name} {robot_namespace} {str(self.initial_pose)} \
            && ros2 launch -d /workdir/launch/nav2_bringup.launch.py" """
            #self.command = f"""bash -c "python3 /workdir/launch/robot_bringup.py {robot_name} {robot_namespace} {str(self.initial_pose)} \
            #&& (ros2 launch /workdir/launch/turtlebot3_state_publisher.launch.py & ros2 launch -d /workdir/launch/nav2_bringup.launch.py)" """

        self.add_mount(source=launch_path, target="/workdir/launch")
        self.add_mount(source=param_path, target="/workdir/param")
        self.add_mount(source=models_path, target="/workdir/models")