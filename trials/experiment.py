import queue
from docker import DockerClient

from trials.trial import Trial
from trials.hmrs_trial import HMRSTrial
from core.components import Component
from queue import Queue


class Experiment(Component):
    def __init__(self, trial_list: [Trial], name: str = ""):
        super(Experiment, self).__init__()
        self.trial_list = trial_list
        self.trial_queue = Queue()
        for trial in trial_list:
            self._add(trial)
            self.trial_queue.put(trial)
        self.name = name

    @classmethod
    def from_config(cls, docker_client: DockerClient, config, map_path: str, param_path: str, path_to_world: str,
                    name: str = "", *trial_args, **trial_kwargs):
        trial_list = []
        for trial_config in config:
            trial = HMRSTrial(docker_client, trial_config, trial_config['id'], trial_config['code'],
                              path_to_world=path_to_world, *trial_args, **trial_kwargs)
            trial.sim.add_mount(source=map_path, target="/workdir/map")
            trial.setup_robots(param_path=param_path,
                               map_yaml='/workdir/param/map/map.yaml',
                               use_pose_logger=True, use_battery=True)
            trial.setup_nurse()
            trial_list.append(trial)

        return cls(trial_list, name)

    def build(self):
        for trial in self.trial_list:
            trial.build()

    def run(self):
        self.run_next_trial()

    def run_next_trial(self):
        try:
            trial = self.trial_queue.get(block=False)
            trial.run()
        except queue.Empty:
            print('Done executing all trials.')
