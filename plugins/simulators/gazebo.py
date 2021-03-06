import docker
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/Gazebo/")


class Gazebo(Simulator):
    def __init__(self, tag: str = "gazebo", path: str = DEFAULT_PATH,
                 headless_command="ros2 launch --debug /workdir/launch/sim_launch.py",
                 gui_command='ros2 launch /workdir/launch/sim_launch.py headless:=False', *args, **kwargs):
        super(Gazebo, self).__init__(headless_command=headless_command, gui_command=gui_command, path=path, tag=tag,
                                     *args, **kwargs)

        if path == DEFAULT_PATH:
            self.add_mount(f"{DEFAULT_PATH}/launch:/workdir/launch")

    def run(self, network_mode="host", **run_kwargs):
        if network_mode:
            run_kwargs['network_mode'] = "host"
        super(Gazebo, self).run(**run_kwargs)
