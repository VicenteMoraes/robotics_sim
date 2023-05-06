from docker import DockerClient
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/GazeboIgnition/")


class GazeboIgnition(Simulator):
    def __init__(self, docker_client: DockerClient, tag: str = "gazebo_ignition", path: str = DEFAULT_PATH,
                 headless: bool = True, headless_command='gz sim -v 4 -s -r empty.sdf',
                 gui_command='gz sim -v 4 empty.sdf', *args, **kwargs):
        super(GazeboIgnition, self).__init__(docker_client=docker_client, headless_command=headless_command,
                                             gui_command=gui_command, headless=headless, path=path, tag=tag,
                                             *args, **kwargs)

        self.headless = headless

        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        if not headless:
            self.env['NVIDIA_VISIBLE_DEVICES'] = 'all'
            self.env['NVIDIA_DRIVER_CAPABILITIES'] = 'all'
            self.env['MESA_GL_VERSION_OVERRIDE'] = '4.2'

        if path == DEFAULT_PATH:
            self.add_mount(source=f"{DEFAULT_PATH}/launch", target="/workdir/launch")

    def run(self, network_mode=False, **run_kwargs):
        if network_mode:
            run_kwargs['network_mode'] = "host"
        if not self.headless:
            run_kwargs['runtime'] = 'nvidia'
        super(GazeboIgnition, self).run(**run_kwargs)

    def add_model_path(self, container, path: str, name: str = 'model_path'):
        mnt = container.add_mount(source=name, target=path, mount_type='volume')
        self.mounts.append(mnt)
        self.env['IGN_FILE_PATH'] = path+"/models"
