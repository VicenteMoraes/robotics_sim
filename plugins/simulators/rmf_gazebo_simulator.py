import docker
from plugins.simulators.simulator import Simulator
from core.components import ProjectPath


class RMFGazebo(Simulator):
    def __init__(self, docker_client: docker.DockerClient, headless: bool = True, priority: int = 5,
                 tag: str = "rmfgazebo", path: str = str(ProjectPath/"dockerfiles/RMFGazebo/"),
                 dockerfile: str = "Dockerfile", auto_remove: bool = True):
        super(RMFGazebo, self).__init__(docker_client=docker_client, path=path,
                                        headless=headless, priority=priority, tag=tag, dockerfile=dockerfile,
                                        auto_remove=auto_remove)
        self.tag = tag
        self.env["ROS_DOMAIN_ID"] = "9"

    def command(self):
        if self.headless:
            return "ros2 launch rmf_demos_gz office.launch.xml headless:=1"
        return 'bash -c "xauth add $XAUTH_LIST && ros2 launch rmf_demos_gz office.launch.xml"'

    def run(self, network_mode: str = "host", **run_kwargs):
        super(RMFGazebo, self).run(network_mode=network_mode, **run_kwargs)

