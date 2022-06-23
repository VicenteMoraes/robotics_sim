from plugins.plugin import DockerPlugin
from core.envs.baseenv import Environment
from abc import abstractmethod
import subprocess
import os


class Simulator(DockerPlugin):
    def __init__(self, docker_client, path: str, headless: bool = True, priority: int = 5, container_name: str = "",
                  tag: str = "sim", dockerfile: str = "Dockerfile", auto_remove: bool = True):
        super(Simulator, self).__init__(docker_client, priority)
        self.headless = headless
        self.path = path # Path to dockerfile directory
        self.tag = tag
        self.dockerfile = dockerfile # Name of dockerfile file
        self.auto_remove = auto_remove
        self.env = Environment()
        self.container_name = container_name if container_name else f"{self.tag}_container"
        if not self.headless:
            self.load_gui_env()
            self.add_mount("/tmp/.X11-unix/:/tmp/.X11-unix/")

    def load_gui_env(self):
        self.env["DISPLAY"] = os.environ['DISPLAY']
        self.env["QT_GRAPHICSSYSTEM"] = "native"
        self.env["QT_X11_NO_MITSHM"] = "1"
        self.env["XAUTH_LIST"] = subprocess.run(["xauth", "list"], text=True, capture_output=True).stdout

    @abstractmethod
    def command(self):
        pass

    def build(self, **build_kwargs):
        self.image = self.docker_client.images.build(path=self.path, dockerfile=self.dockerfile, tag=self.tag,
                                                     **build_kwargs)

    def run(self, **run_kwargs):
        self.container = self.docker_client.containers.run(image=self.tag, command=self.command(), detach=True,tty=True,
                                                           name=self.container_name, mounts=self.mounts,
                                                           environment=self.env.to_list(), auto_remove=self.auto_remove,
                                                           **run_kwargs)