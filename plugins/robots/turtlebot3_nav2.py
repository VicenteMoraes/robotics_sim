from plugins.robots.turtlebot3 import Turtlebot3
from docker import DockerClient
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Robots/Turtlebot3_nav2")


class Turtlebot3withNav2(Turtlebot3):
    def __init__(self, docker_client: DockerClient, docker_path: str = DEFAULT_PATH, tag: str = "turtlebot3_nav2",
                 command: str = None, launch_path: str = f"{DEFAULT_PATH}/launch", param_path: str = f"{DEFAULT_PATH}/param",
                 models_path: str = f"{DEFAULT_PATH}/models", turtlebot3_model: str = 'burger', lds_model: str = 'LDS-01',
                 robot_name: str = "turtlebot", robot_namespace: str = "turtlebot", use_rviz: bool = False,
                 map_yaml: str = '', params_yaml: str = '', *args, **kwargs):
        super(Turtlebot3, self).__init__(docker_client=docker_client, path=docker_path, tag=tag, command=command,
                                         robot_name=robot_name, *args, **kwargs)

        self.env['TURTLEBOT3_MODEL'] = turtlebot3_model
        self.env['LDS_MODEL'] = lds_model
        self.env['MAP_YAML'] = map_yaml
        self.env['PARAMS_YAML'] = params_yaml
        self.use_rviz = use_rviz
        self.env['USE_RVIZ'] = str(use_rviz)

        if self.use_rviz:
            self.load_gui()

        if command is None:
            self.command = f"""bash -c "ros2 launch -d /workdir/launch/nav2_bringup.launch.py \
            & python3 /workdir/launch/battery.py" """

        self.add_mount(source=launch_path, target="/workdir/launch")
        self.add_mount(source=param_path, target="/workdir/param")
        self.add_mount(source=models_path, target="/workdir/models")

    def run(self, **run_kwargs):
        if self.use_rviz:
            run_kwargs['runtime'] = 'nvidia'
        super(Turtlebot3withNav2, self).run(**run_kwargs)