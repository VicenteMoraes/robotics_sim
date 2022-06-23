from core.envs.baseenv import Environment


def test_to_list():
    env = Environment()
    env["test"] = 1
    lst = env.to_list()
    assert lst[0] == "test=1"
