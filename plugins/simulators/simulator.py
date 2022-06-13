from plugins.plugin import DockerPlugin
from core.envs.baseenv import Environment
from abc import abstractmethod
import subprocess


class Simulator(DockerPlugin):
    def __init__(self, docker_client, headless: bool = True, priority: int = 5):
        super(Simulator, self).__init__(docker_client, priority)
        self.headless = headless
        self.env = Environment()
        if not self.headless:
            self.load_gui_env()

    def load_gui_env(self):
        self.env["DISPLAY"] = ""
        self.env["QT_GRAPHICSSYSTEM"] = "native"
        self.env["QT_X11_NO_MITSHM"] = "1"
        self.env["ROS_DOMAIN_ID"] = "9"
        self.env["XAUTH_LIST"] = subprocess.check_output(["xauth", "list"])

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def build(self):
        pass
