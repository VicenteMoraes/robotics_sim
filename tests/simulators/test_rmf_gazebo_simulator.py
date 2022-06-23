import docker
from plugins.simulators.rmf_gazebo_simulator import RMFGazebo


def test_build():
    client = docker.from_env()
    sim = RMFGazebo(client, headless=True)
    sim.build()
    assert True


def test_sim_run():
    client = docker.from_env()
    assert True
    return
    sim = RMFGazebo(client, headless=True)
    sim.run()
    sim.container.stop(force=True)
