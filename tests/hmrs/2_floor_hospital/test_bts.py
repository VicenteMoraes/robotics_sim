import docker
from core.pose import Pose, quaternion_from_euler
from core.components import ProjectPath
from plugins.simulators.gazebo import Gazebo
from plugins.robots.turtlebot3 import Turtlebot3
from plugins.robots.turtlebot3_nav2 import Turtlebot3withNav2
from plugins.networks.ros2_network import ROS2Network
from plugins.ros2.rviz import RVIZ


def test_behavior_tree():
    client = docker.from_env()
    network = ROS2Network(docker_client=client, name="ros2")
    sim = Gazebo(docker_client=client, headless=True, auto_remove=True, network=network,
                 path_to_world="/workdir/map/2_floor_hospital.world")
    sim.add_mount(source=str(ProjectPath / "tests/hmrs/2_floor_hospital/param/map"), target="/workdir/map")
    sim.add_logger(write_to_file=True, filename="sim.log")
    initial_pose = Pose()
    initial_pose.position.x = 11.5
    initial_pose.position.y = -3.8
    initial_pose.position.z = 6.1
    quat = quaternion_from_euler(0, 0, 1.57)
    initial_pose.orientation.x = quat[0]
    initial_pose.orientation.y = quat[1]
    initial_pose.orientation.z = quat[2]
    initial_pose.orientation.w = quat[3]
    robot = Turtlebot3withNav2(client, robot_name="turtlebot", robot_namespace="turtlebot", auto_remove=True, network=network,
                               initial_pose=initial_pose, map_yaml="/workdir/param/map/second_floor/map.yaml", use_rviz=True,
                               use_battery=False, use_pose_logger=False)
    robot.add_mount(source=str(ProjectPath / "tests/hmrs/2_floor_hospital/param/map"), target="/workdir/param/map")
    robot.add_logger(write_to_file=True, filename="robot.log")

    sim.add_model_path(container=robot, path="/opt/ros/humble/share/turtlebot3_gazebo")

    network.build()
    sim.build()
    robot.build()

    sim.run(network_mode=False)
    robot.run()
