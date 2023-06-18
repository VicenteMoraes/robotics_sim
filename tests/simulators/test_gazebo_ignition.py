from plugins.simulators.gazebo_ignition import GazeboIgnition
import docker


def test_gui():
    return
    client = docker.from_env()
    sim = GazeboIgnition(docker_client=client, headless=False, auto_remove=True)
    sim.add_logger(write_to_file=True, filename="ignition.log")
    sim.build()
    sim.run(network_mode=True, runtime='nvidia')
