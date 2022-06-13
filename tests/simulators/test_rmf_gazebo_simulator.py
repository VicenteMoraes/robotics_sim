import docker
from plugins.simulators.rmf_gazebo_simulator import RMFGazebo


def test_build():
    client = docker.from_env()
    sim = RMFGazebo(client, headless=True)
    sim.build()
    assert True

def test_run():
    client = docker.from_env()
    client.swarm.init()
    sim = RMFGazebo(client, headless=True)
    sim.build()
    sim.run()
    assert True
