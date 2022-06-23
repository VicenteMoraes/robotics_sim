from plugins.simulators.rmf_gazebo_simulator import RMFGazebo
import docker


def test_gui():
    client = docker.from_env()
    sim = RMFGazebo(client, headless=False)
    sim.run()
    assert True
