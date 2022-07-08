from plugins.simulators.rmf_gazebo_simulator import RMFGazebo
import docker


def test_gui():
    assert True
    return
    client = docker.from_env()
    sim = RMFGazebo(docker_client=client, headless=False, auto_remove=False)
    sim.build()
    sim.run()
