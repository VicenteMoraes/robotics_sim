from abc import abstractmethod
from core.components import Component
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
    def __init__(self, docker_client: DockerClient, priority: int = 5):
        super(DockerPlugin, self).__init__(priority)
        self.docker_client = docker_client
        self.image = None
        self.container = None
        self.mounts = []

    def add_mount(self, mount: str):
        mount = mount.split(":")
        source = mount[0]
        target = mount[1]
        mnt = docker.types.Mount(target=target, source=source, type='bind')
        self.mounts.append(mnt)

    @abstractmethod
    def run(self):
        pass

    @abstractmethod
    def build(self):
        pass
