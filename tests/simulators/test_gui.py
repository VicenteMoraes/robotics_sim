from plugins.simulators.gazebo import Gazebo
import docker


def test_gui():
    client = docker.from_env()
    sim = Gazebo(docker_client=client, headless=False, auto_remove=False)
    sim.build()
    sim.run()
    assert True
    return
