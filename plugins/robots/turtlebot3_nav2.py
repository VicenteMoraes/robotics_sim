from plugins.robots.turtlebot3 import Turtlebot3
from docker import DockerClient
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3_nav2")


class Turtlebot3withNav2(Turtlebot3):
    def __init__(self, docker_client: DockerClient, path: str = DEFAULT_PATH, tag: str = "turtlebot3_nav2",
                 command: str = None,
                 turtlebot3_model: str = 'burger', lds_model: str = 'LDS-01',
                 robot_name: str = "turtlebot", robot_namespace: str = "turtlebot", *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=path, tag=tag, command=command,
                                         robot_name=robot_name, *args, **kwargs)

        self.env['TURTLEBOT3_MODEL'] = turtlebot3_model
        self.env['LDS_MODEL'] = lds_model
        self.env['ROBOT_NAMESPACE'] = robot_namespace
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        if command is None:
            self.command = f"""bash -c "python3 /workdir/launch/robot_bringup.py {robot_name} {robot_namespace} {str(self.initial_pose)} \
            && ros2 launch /workdir/launch/turtlebot3_state_publisher.launch.py & ros2 launch -d /workdir/launch/nav2_bringup.launch.py" """
        if path == DEFAULT_PATH:
            self.add_mount(source=f"{DEFAULT_PATH}/launch", target="/workdir/launch")
            self.add_mount(source=f"{DEFAULT_PATH}/param", target="/workdir/param")
            self.add_mount(source=f"{DEFAULT_PATH}/models", target="/workdir/models")
