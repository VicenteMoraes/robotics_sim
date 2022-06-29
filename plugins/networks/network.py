from plugins.plugin import Plugin
import docker


class Network(Plugin):
    def __init__(self, docker_client: docker.DockerClient, priority: int = 5, name: str = "network",
                 driver: str = "bridge", check_duplicate: bool = True, internal: bool = False, **options):
        super(Network, self).__init__(priority)
        self.docker_client = docker_client
        self.name = name
        self.driver = driver
        self.check_duplicate = check_duplicate
        self.internal = internal
        self.options = options

        self.net = None

    def build(self):
        self.net = self.docker_client.networks.create(name=self.name, driver=self.driver,
                                                      internal=self.internal, check_duplicate=self.check_duplicate,
                                                      options=self.options, attachable=True)

    def run(self):
        return