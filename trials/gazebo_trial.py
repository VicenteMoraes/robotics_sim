import docker
from trials.trial import Trial
from plugins.simulators.rmf_gazebo_simulator import RMFGazebo


class GazeboTrial(Trial):
    def __init__(self, trial_id=None, headless: bool = True):
        super(GazeboTrial, self).__init__(trial_id)
        self.docker_client = docker.from_env()
        self.headless = headless
        self.swarm = self.docker_client.swarm.init()

        self.sim = RMFGazebo(client=self.docker_client, headless=headless)
        self.add_plugin(self.sim)

    def run(self):
        super(GazeboTrial, self).run()
        self.docker_client.swarm.leave(force=True)
