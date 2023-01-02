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


def test_trial():
    config = parse_config(str(ProjectPath/"tests/hmrs/experiment/experiment_sample.json"))[0]
    trial = HMRSTrial(config, config['id'], config['code'], headless=True, path_to_world="/workdir/map/hospital.world",
                      use_rviz=True)
    trial.sim.add_mount(source=str(ProjectPath/"tests/hmrs/param/map"), target="/workdir/map")
    trial.setup_robots(param_path=str(ProjectPath/"tests/hmrs/param"), map_yaml='/workdir/param/map/map.yaml')
    trial.setup_nurse()
    trial.build()
    trial.run()


def test_hmrsim():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/workdir/map/hospital.world")
    sim._add(DockerLogger(target='', write_to_file=True, filename='sim.log', timeout=300))
    sim.add_mount(source=str(ProjectPath/"tests/hmrs/map"), target="/workdir/map")
    ps = pose.Pose()
    ps.position.x = -2
    ps.position.y = -0.5
    ps.position.z = 0.1
    #robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="robot1", auto_remove=True, network=network, initial_pose=ps)
    #robot._add(DockerLogger(target='', write_to_file=True, filename='robot.log', timeout=300))
    #sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
    #robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2", auto_remove=True, network=network,
    #                            initial_pose=ps)
    rviz = RVIZ(client, auto_remove=True, network=network)
    rviz._add(DockerLogger(target='', write_to_file=True, filename='rviz.log', timeout=300))

    network.build()
    sim.build()
    #robot.build()
    rviz.build()
    #robot2.build()

    sim.run(network_mode=False)
    #robot.run()
    #robot2.run()
    rviz.run()
