import docker
from modules.simulators.gazebo import Gazebo
from modules.robots.turtlebot3_nav2 import Turtlebot3withNav2
from modules.networks.ros2_network import ROS2Network
from modules.loggers.docker_logger import DockerLogger
from core import pose


def test_turtlebot3withnav2():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world")
    sim.add(DockerLogger(target='', write_to_file=True, filename='sim.log', timeout=300, update_interval=1))
    ps = pose.Pose()
    ps.position.x = -2
    ps.position.y = -0.5
    ps.position.z = 0.1
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network,
                               initial_pose=ps, use_rviz=True)
    robot.add(DockerLogger(target='', write_to_file=True, filename='robot.log', timeout=300))
    sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")
    ps = pose.Pose()
    ps.position.x = 2
    ps.position.y = -1
    ps.position.z = 0.1
    robot2 = Turtlebot3withNav2(client, robot_name="turtlebot2", robot_namespace="turtlebot2", container_name="turtlebot2",
                                auto_remove=True, network=network, initial_pose=ps, use_rviz=True)
    robot2.add(DockerLogger(target='', write_to_file=True, filename='robot2.log', timeout=300))

    network.build()
    sim.build()
    robot.build()
    robot2.build()

    sim.run(network_mode=False)
    robot.run()
    robot2.run()
