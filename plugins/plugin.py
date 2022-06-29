from abc import abstractmethod
from core.components import Component
from core.envs.baseenv import Environment
from docker import DockerClient
import docker


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

    def __repr__(self):
        try:
            return f"{str(self)} plugin attached to Trial {str(self.parent)}, ID: {self.parent.trial_id}"
        except AttributeError:
            return f"{str(self)} plugin attached to Parent {str(self.parent)}, ID: {self.parent.uuid}"

    def __ge__(self, other):
        return self.priority >= other.priority

    def __le__(self, other):
        return self.priority <= other.priority


class DockerPlugin(Plugin):
    def __init__(self, docker_client: DockerClient, path: str, command: str, priority: int = 5, tag: str = "",
                 dockerfile: str = "Dockerfile", auto_remove: bool = False, container_name: str = "", network=None):
        super(DockerPlugin, self).__init__(priority)
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

    def add_mount(self, mount: str):
        mount = mount.split(":")
        source = mount[0]
        target = mount[1]
        mnt = docker.types.Mount(target=target, source=source, type='bind')
        self.mounts.append(mnt)

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
