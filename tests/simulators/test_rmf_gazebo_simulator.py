import docker
from plugins.simulators.rmf_gazebo_simulator import RMFGazebo


def test_build():
    return
    client = docker.from_env()
    sim = RMFGazebo(client, headless=True)
    sim.build()


def test_sim_run():
    return
    client = docker.from_env()
    sim = RMFGazebo(client, headless=True)
    sim.run()
    sim.container.stop(force=True)
