import docker
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath


class RMFGazebo(Simulator):
    def __init__(self, docker_client: docker.DockerClient, headless: bool = True, priority: int = 5,
                 tag: str = "rmfgazebo"):
        super(RMFGazebo, self).__init__(docker_client, headless, priority)
        self.tag = tag
        self.command = f'bash -c "xauth add $XAUTH_LIST && ros2 launch rmf_demos_gz office.launch.xml headless:={int(not headless)}"'

    def build(self):
        self.image = self.docker_client.images.build(path=str(ProjectPath/"dockerfiles/RMFGazebo/"),
                                                     dockerfile="Dockerfile", tag=self.tag)
        if not self.headless:
            #TODO Implement GUI volume bind
            pass

    def run(self):
        self.service = self.docker_client.services.create(image=self.tag, env=self.env.to_list(),
                                                          command=self.command)
