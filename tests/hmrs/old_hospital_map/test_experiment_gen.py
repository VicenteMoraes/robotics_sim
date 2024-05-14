from modules.hmrs.load_experiment import parse_config
from core.components import ProjectPath
from trials.hmrs_trial import HMRSTrial

def test_generate_trial():
    return
    trial_config = parse_config(ProjectPath/"tests/hmrs/experiment/experiment_sample.json")
    for config in trial_config:
        trial = HMRSTrial(config, config['id'])
        trial.setup_robots()