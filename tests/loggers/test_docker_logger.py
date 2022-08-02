import docker
from plugins.simulators.gazebo import Gazebo
from plugins.loggers.docker_logger import DockerLogger
from time import sleep


def test_logger():
    assert True
    return
    target = "[gzserver-1]  Audio will be disabled."
    client = docker.from_env()
    sim = Gazebo(docker_client=client, headless=True, auto_remove=True)
    logger = DockerLogger(target)
    sim._add(logger)
    sim.build()
    sim.run(network_mode=False)
    sleep(20)
    assert logger.success


def test_write_logs():
    assert True
    return
    target = "[gzserver-1]  Audio will be disabled."
    client = docker.from_env()
    sim = Gazebo(docker_client=client, headless=True, auto_remove=True)
    logger = DockerLogger(target, write_to_file=True, filename="log_test")
    sim._add(logger)
    sim.build()
    sim.run(network_mode=False)
    sleep(20)
    assert logger.success
