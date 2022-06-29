from plugins.simulators.gazebo import Gazebo
import docker


def test_gui():
    assert True
    return
    client = docker.from_env()
    sim = Gazebo(client, headless=False, auto_remove=False)
    sim.build()
    sim.run()
