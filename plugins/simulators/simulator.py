from plugins.plugin import DockerPlugin
import subprocess
import os


class Simulator(DockerPlugin):
    def __init__(self, headless_command: str, gui_command: str, headless: bool = True, tag: str = "sim",
                 path_to_world: str = "", world_arg: str = "world_map", *args, **kwargs):
        super(Simulator, self).__init__(tag=tag, command="", *args, **kwargs)
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