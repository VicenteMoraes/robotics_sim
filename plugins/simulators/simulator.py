from plugins.plugin import DockerPlugin
import subprocess
import os


class Simulator(DockerPlugin):
    def __init__(self, docker_client, path: str, headless_command: str, gui_command: str, network=None,
                 headless: bool = True, priority: int = 5, container_name: str = "",
                 tag: str = "sim", dockerfile: str = "Dockerfile", auto_remove: bool = True, path_to_world: str = "",
                 world_arg: str = "world_map"):
        super(Simulator, self).__init__(docker_client=docker_client, path=path, priority=priority, network=network,
                                        container_name=container_name, tag=tag, dockerfile=dockerfile,
                                        auto_remove=auto_remove, command="")
        self.headless = headless

        if not self.headless:
            self.command = gui_command
            self.load_gui_env()
            self.add_mount("/tmp/.X11-unix/:/tmp/.X11-unix/")
        else:
            self.command = headless_command

        if path_to_world:
            self.command += ' ' + f"{world_arg}:={path_to_world}"

    def load_gui_env(self):
        self.env["DISPLAY"] = os.environ['DISPLAY']
        self.env["QT_GRAPHICSSYSTEM"] = "native"
        self.env["QT_X11_NO_MITSHM"] = "1"
        self.env["XAUTHORITY"] = "/tmp/.docker.xauth"
        self.env["XAUTH_LIST"] = subprocess.run(["xauth", "list"], text=True, capture_output=True).stdout