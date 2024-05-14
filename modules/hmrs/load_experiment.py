import json


def parse_config(file: str):
    with open(file) as fp:
        return json.load(fp)


def generate_trial(file: str):
    trials_config = parse_config(file)
    for config in trials_config:
        pass