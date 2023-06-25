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
            self.add(trial)
            self.trial_queue.put(trial)
        self.name = name

    @classmethod
    def from_config(cls, docker_client: DockerClient, config, map_path: str, param_path: str, path_to_world: str,
                    name: str = "", trials_to_execute: [str] = None, *trial_args, **trial_kwargs):
        trial_list = []
        for trial_config in config:
            trial = HMRSTrial(docker_client, trial_config, trial_config['id'], trial_config['code'],
                              path_to_world=path_to_world, *trial_args, **trial_kwargs)
            if trials_to_execute is not None:
                if trial.trial_id not in trials_to_execute:
                    continue
            trial.sim.add_mount(source=map_path, target="/workdir/map")
            trial.setup_robots(param_path=param_path,
                               map_yaml='/workdir/param/map/map.yaml',
                               use_pose_logger=True, use_battery=True)
            trial.setup_nurse()
            trial_list.append(trial)

        return cls(trial_list, name)

    def build(self):
        pass

    def run(self, number_of_trials: int = 1, *run_args, **run_kwargs):
        for _ in range(number_of_trials):
            self.run_next_trial(*run_args, **run_kwargs)

    def run_next_trial(self, *run_args, **run_kwargs):
        try:
            trial = self.trial_queue.get(block=False)
            trial.build()
            print(f"Running Trial: {trial.trial_id}")
            trial.run(*run_args, **run_kwargs)
        except queue.Empty:
            print('Done executing all trials.')

    def event_callback(self, msg: str):
        match msg:
            case "RUN NEXT TRIAL":
                self.run_next_trial()
            case _:
                super(Experiment, self).event_callback(msg)
