from abc import abstractmethod
from core.components import Component
from core.envs.baseenv import Environment
from docker import DockerClient
from plugins.loggers.docker_logger import DockerLogger
import docker
import os
import subprocess


class Plugin(Component):
    def __init__(self, priority: int = 5):
        # Priorities: 0 - highest, 5 - lowest
        super(Plugin, self).__init__()
        self.priority = priority

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def build(self):
        pass

    def stop(self):
        pass

    def __repr__(self):
        try:
            return f"{str(self)} plugin attached to Trial {str(self.parent)}, ID: {self.parent.trial_id}"
        except AttributeError:
            return f"{str(self)} plugin attached to Parent {str(self.parent)}, ID: {self.parent.uuid}"

    def __gt__(self, other):
        return self.priority > other.priority

    def __lt__(self, other):
        return self.priority < other.priority

    def __eq__(self, other):
        return self.priority == other.priority


class DockerPlugin(Plugin):
    def __init__(self, docker_client: DockerClient, path: str, command: str, tag: str,
                 dockerfile: str = "Dockerfile", auto_remove: bool = True, container_name: str = "", network=None,
                 *args, **kwargs):
        super(DockerPlugin, self).__init__(*args, **kwargs)
        self.docker_client = docker_client
        self.path = path  # Path to dockerfile directory
        self.tag = tag
        self.dockerfile = dockerfile  # Name of dockerfile file
        self.auto_remove = auto_remove
        self.image = None
        self.container = None
        self.command = command
        self.network = network
        self.mounts = []
        self.env = Environment()
        self.env["TAG"] = self.tag
        self.container_name = container_name if container_name else f"{self.tag}_container"

    def add_mount(self, source: str, target: str, mount_type: str = 'bind', **mount_kwargs):
        mnt = docker.types.Mount(target=target, source=source, type=mount_type, **mount_kwargs)
        self.mounts.append(mnt)
        return mnt

    def load_gui(self):
        self.load_gui_env()
        self.add_mount(source="/tmp/.X11-unix/", target="/tmp/.X11-unix/")

    def load_gui_env(self):
        self.env["DISPLAY"] = os.environ['DISPLAY']
        self.env["QT_GRAPHICSSYSTEM"] = "native"
        self.env["QT_X11_NO_MITSHM"] = "1"
        #self.env["XAUTHORITY"] = "/tmp/.docker.xauth"
        self.env["XAUTH_LIST"] = subprocess.run(["xauth", "list"], text=True, capture_output=True).stdout

    def connect(self, network):
        network.connect(self.container)

    def run(self, **run_kwargs):
        if self.network:
            run_kwargs['network'] = self.network.name
        self.container = self.docker_client.containers.run(image=self.tag, command=self.command, detach=True, tty=True,
                                                           name=self.container_name, mounts=self.mounts,
                                                           environment=self.env.to_list(), auto_remove=self.auto_remove,
                                                           **run_kwargs)

    def build(self, **build_kwargs):
        self.image = self.docker_client.images.build(path=self.path, dockerfile=self.dockerfile, tag=self.tag,
                                                     **build_kwargs)

    def add_logger(self, target: str = "", write_to_file: bool = False, filename: str = '', update_interval: float = 1,
                   log_args: list = None, timeout: float = 15*60, *logger_args, **logger_kwargs):
        self._add(DockerLogger(target=target, write_to_file=write_to_file, filename=filename,
                               update_interval=update_interval, log_args=log_args, timeout=timeout,
                               *logger_args, **logger_kwargs))

    def kill(self, signal=None):
        self.container.kill(signal)

    def stop(self):
        try:
            self.container.stop()
            for child in self.children:
                child.stop()
        except AttributeError:
            pass
        except docker.errors.NotFound:
            pass

