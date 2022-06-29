import docker
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Gazebo/")


class Gazebo(Simulator):
    def __init__(self, docker_client: docker.DockerClient, headless: bool = True, priority: int = 5,
                 tag: str = "gazebo", path: str = DEFAULT_PATH, network=None,
                 headless_command="ros2 launch --debug /workdir/launch/sim_launch.py",
                 gui_command='ros2 launch /workdir/launch/sim_launch.py headless:=False',
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, path_to_world: str = ""):
        super(Gazebo, self).__init__(docker_client=docker_client, headless_command=headless_command, network=network,
                                     gui_command=gui_command, path=path, auto_remove=auto_remove,
                                     headless=headless, priority=priority, tag=tag, dockerfile=dockerfile,
                                     path_to_world=path_to_world)

        if path == DEFAULT_PATH:
            self.add_mount(f"{DEFAULT_PATH}/launch:/workdir/launch")

    def run(self, network_mode="host", **run_kwargs):
        if self.headless:
            super(Gazebo, self).run(**run_kwargs)
        else:
            super(Gazebo, self).run(network_mode=network_mode, **run_kwargs)

