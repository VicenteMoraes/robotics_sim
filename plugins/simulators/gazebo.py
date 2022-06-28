import docker
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath


class Gazebo(Simulator):
    def __init__(self, docker_client: docker.DockerClient, headless: bool = True, priority: int = 5,
                 tag: str = "gazebo", path: str = str(ProjectPath/"dockerfiles/Gazebo/"),
                 headless_command="ros2 launch --debug /workdir/launch/sim_launch.py",
                 gui_command='ros2 launch /workdir/launch/sim_launch.py headless:=False',
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, path_to_world: str = ""):
        super(Gazebo, self).__init__(docker_client=docker_client, headless_command=headless_command,
                                     gui_command=gui_command, path=path, auto_remove=auto_remove,
                                     headless=headless, priority=priority, tag=tag, dockerfile=dockerfile,
                                     path_to_world=path_to_world)

        self.tag = tag
        self.path_to_world = path_to_world
        self.add_mount(f"{ProjectPath/'dockerfiles/Gazebo/launch'}:/workdir/launch")

    def run(self, network_mode="host", **run_kwargs):
        super(Gazebo, self).run(network_mode="host", **run_kwargs)

