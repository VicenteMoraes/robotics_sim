import docker
from modules.simulators.gazebo import Gazebo
from modules.robots.turtlebot3_nav2 import Turtlebot3withNav2
from modules.networks.ros2_network import ROS2Network
from core import pose
from core.components import ProjectPath
from modules.hmrs.load_experiment import parse_config
from trials.hmrs_trial import HMRSTrial
from trials.experiment import Experiment
import os
from core.components import ProjectPath


def test_full_experiment():
    return
    hosts = [f"les-0{i}" for i in range(1, 9)]
    config = parse_config(str(ProjectPath/"tests/hmrs/old_hospital_map/experiment/trials.json"))
    experiments = []
    docker_clients = []
    for host in hosts:
        client = docker.DockerClient(base_url=f'ssh://lesunb@{host}')
        docker_clients.append(client)
        experiments.append(Experiment.from_config(client, config=config,
                                                  map_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param/map"),
                                                  param_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param"),
                                                  use_rviz=False, path_to_world="/workdir/map/hospital.world",
                                                  dir=f"distributed_experiment/{host}", headless=True, ssh_host=f"lesunb@{host}"))

    for experiment in experiments:
        experiment.build()
    for num, experiment in enumerate(experiments):
        print(f'Running Host: {hosts[num]}')
        experiment.run()


def test_experiment():
    docker_client = docker.from_env()
    config = parse_config(str(ProjectPath/"tests/hmrs/old_hospital_map/experiment/trials.json"))
    experiment = Experiment.from_config(docker_client, config=config, map_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param/map"),
                                        param_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param"), use_rviz=True,
                                        path_to_world="/workdir/map/hospital.world", dir="experiment",
                                        headless=False, trials_to_execute=["51_abcbcp"])
    experiment.build()
    experiment.run()


def test_trial():
    return
    docker_client = docker.from_env()
    config = parse_config(str(ProjectPath/"tests/hmrs/old_hospital_map/experiment/experiment_sample.json"))[0]
    trial = HMRSTrial(docker_client, config, config['id'], config['code'], headless=False, path_to_world="/workdir/map/hospital.world",
                      use_rviz=True)
    trial.sim.add_mount(source=str(ProjectPath/"tests/hmrs/old_hospital_map/param/map"), target="/workdir/map")
    trial.setup_robots(param_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param"), map_yaml='/workdir/param/map/map.yaml',
                       use_pose_logger=True, use_battery=True)
    trial.setup_nurse()
    trial.build()
    trial.run()


def test_hmrsim():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/workdir/map/hospital.world")
    sim.add_logger(write_to_file=True, filename="sim.log")
    sim.add_mount(source=str(ProjectPath/"tests/hmrs/old_hospital_map/param/map"), target="/workdir/map")
    ps = pose.Pose()
    ps.position.x = 0
    ps.position.y = 0
    ps.position.z = 0.1
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="", auto_remove=True, network=network, initial_pose=ps,
                               use_rviz=True, use_slam=True)
    robot.add_logger(write_to_file=True, filename="robot.log")
    sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
    #robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2", auto_remove=True, network=network,
    #                            initial_pose=ps)

    network.build()
    sim.build()
    robot.build()
    #robot2.build()

    sim.run(network_mode=False)
    robot.run()
    #robot2.run()


if __name__ == "__main__":
    print('Started')
    test_full_experiment()
