import docker
from plugins.simulators.gazebo import Gazebo
from plugins.loggers.docker_logger import DockerLogger
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.ros2.skill_library import SkillLibrary
from plugins.networks.ros2_network import ROS2Network
from core import pose


def test_skill_library():
    return
    client = docker.from_env()
    network = ROS2Network(client, name="ros2")
    sim = Gazebo(client, headless=False, auto_remove=True, network=network, path_to_world="/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_world.world")
    sim.add(DockerLogger(target='', write_to_file=True, filename='sim.log', timeout=300))
    ps = pose.Pose()
    ps.position.x = -2
    ps.position.y = -0.5
    ps.position.z = 0.1
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network,
                               initial_pose=ps, use_rviz=True)
    robot.add(DockerLogger(target='', write_to_file=True, filename='robot.log', timeout=300))
    sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")

    lib = SkillLibrary(client, '', network=network, robot_name='turtlebot', robot_namespace='turtlebot', auto_remove=True)
    lib.add_logger(write_to_file=True, filename='skill_library.log')

    network.build()
    sim.build()
    robot.build()

    sim.run(network_mode=False)
    robot.run()
