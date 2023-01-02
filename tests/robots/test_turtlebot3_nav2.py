import docker
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ
from plugins.loggers.docker_logger import DockerLogger
from core import pose


def test_turtlebot3withnav2():
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world")
    sim._add(DockerLogger(target='', write_to_file=True, filename='sim.log', timeout=300))
    ps = pose.Pose()
    ps.position.x = -2
    ps.position.y = -0.5
    ps.position.z = 0.1
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network,
                               initial_pose=ps, use_rviz=True)
    robot._add(DockerLogger(target='', write_to_file=True, filename='robot.log', timeout=300))
    sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
    ps = pose.Pose()
    ps.position.x = 0
    ps.position.y = -1
    ps.position.z = 0.1
    robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2",
                                auto_remove=True, network=network, initial_pose=ps, use_rviz=True)
    robot._add(DockerLogger(target='', write_to_file=True, filename='robot2.log', timeout=300))

    network.build()
    sim.build()
    robot.build()
    robot2.build()

    sim.run(network_mode=False)
    robot.run()
    robot2.run()
