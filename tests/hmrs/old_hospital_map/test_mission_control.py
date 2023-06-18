import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ
from plugins.loggers.docker_logger import DockerLogger
from core import pose
from core.components import ProjectPath
from plugins.hmrs.load_experiment import parse_config
from trials.hmrs_trial import HMRSTrial
from trials.experiment import Experiment


def test_experiment():
    return
    docker_client = docker.from_env()
    config = parse_config(str(ProjectPath/"tests/hmrs/old_hospital_map/experiment/trials.json"))
    experiment = Experiment.from_config(docker_client, config=config, map_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param/map"),
                                        param_path=str(ProjectPath/"tests/hmrs/old_hospital_map/param"), use_rviz=True,
                                        path_to_world="/workdir/map/hospital.world", dir="experiment", headless=False)
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
