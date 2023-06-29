from plugins.plugin import Module
import docker


class Network(Module):
    def __init__(self, docker_client: docker.DockerClient, priority: int = 1, name: str = "network",
                 driver: str = "bridge", check_duplicate: bool = False, internal: bool = False, **options):
        super(Network, self).__init__(priority)
        self.docker_client = docker_client
        self.name = name
        self.driver = driver
        self.check_duplicate = check_duplicate
        self.internal = internal
        self.options = options

        self.net = None

    def build(self):
        try:
            self.net = self.docker_client.networks.create(name=self.name, driver=self.driver,
                                                          internal=self.internal, check_duplicate=self.check_duplicate,
                                                          options=self.options, attachable=True)
        except docker.errors.APIError as e:
            if not self.check_duplicate and e.explanation == f'network with name {self.name} already exists':
                pass

    def run(self):
        return