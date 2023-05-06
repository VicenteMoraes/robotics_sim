from plugins.plugin import DockerPlugin
from docker import DockerClient


class Simulator(DockerPlugin):
    def __init__(self, docker_client: DockerClient, path: str, headless_command: str, gui_command: str, headless: bool = True,
                 tag: str = "sim", path_to_world: str = "", world_arg: str = "world_map", *args, **kwargs):
        super(Simulator, self).__init__(docker_client=docker_client, path=path, tag=tag, command="", *args, **kwargs)
        self.headless = headless
        self.env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

        if not self.headless:
            self.command = gui_command
            self.load_gui()
        else:
            self.command = headless_command

        if path_to_world:
            self.command += ' ' + f"{world_arg}:={path_to_world}"
