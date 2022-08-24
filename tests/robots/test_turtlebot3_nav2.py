import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ
from plugins.loggers.docker_logger import DockerLogger
from core import pose


def test_turtlebot3withnav2():
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network)
    sim._add(DockerLogger(target='', write_to_file=True, filename='sim.log', timeout=300))
    ps = pose.Pose()
    ps.position.x = 2
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="robot1", auto_remove=True, network=network, initial_pose=ps)
    robot._add(DockerLogger(target='', write_to_file=True, filename='robot.log', timeout=300))
    #robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2", auto_remove=True, network=network,
    #                            initial_pose=ps)
    rviz = RVIZ(client, auto_remove=True, network=network)
    rviz._add(DockerLogger(target='', write_to_file=True, filename='rviz.log', timeout=300))

    network.build()
    sim.build()
    robot.build()
    rviz.build()
    #robot2.build()

    sim.run(network_mode=False)
    robot.run()
    #robot2.run()
    rviz.run()
