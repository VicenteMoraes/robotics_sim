from docker import DockerClient
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath

DEFAULT_PATH = str(ProjectPath/"dockerfiles/GazeboIgnition/")


class GazeboIgnition(Simulator):
    def __init__(self, docker_client: DockerClient, tag: str = "gazebo_ignition", path: str = DEFAULT_PATH,
                 headless: bool = True, headless_command='ign gazebo -v 4 -s -r empty.sdf', hostname: str = 'gz_sim',
                 gui_command='ros2 launch /workdir/launch/gazebo_bringup.launch.py',  *args, **kwargs):
        super(GazeboIgnition, self).__init__(docker_client=docker_client, headless_command=headless_command,
                                             gui_command=gui_command, headless=headless, path=path, tag=tag,
                                             *args, **kwargs)

        self.hostname = hostname
        self.env['IGN_GAZEBO_RESOURCE_PATH'] = '/opt/ros/humble/share/'

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
        super(GazeboIgnition, self).run(hostname=self.hostname, **run_kwargs)

    def add_model_path(self, container, path: str, name: str = 'ign_model_path'):
        mnt = container.add_mount(source=name, target=path, mount_type='volume')
        self.mounts.append(mnt)
        try:
            self.env['IGN_GAZEBO_RESOURCE_PATH'] += f":{path}"
        except KeyError:
            self.env['IGN_GAZEBO_RESOURCE_PATH'] = path

    def add_gui_plugin_path(self, container, path: str, name: str = 'gui_plugin_path'):
        mnt = container.add_mount(source=name, target=path, mount_type='volume')
        self.mounts.append(mnt)
        self.env['IGN_GUI_PLUGIN_PATH'] = path+"/models"
