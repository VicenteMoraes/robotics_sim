from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    headless = DeclareLaunchArgument(
        name="headless",
        default_value="True",
        description="Run headless?"
    )
    world_map = DeclareLaunchArgument(
        name="world_map",
        default_value="",
        description="World model file to run inside gazebo"
    )
    gz = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so',
             LaunchConfiguration("world_map")]
    )
    gz_gui = ExecuteProcess(
        cmd=['gzclient', '--verbose', LaunchConfiguration("world_map")],
        condition=UnlessCondition(LaunchConfiguration("headless"))
    )
    return LaunchDescription([headless, world_map, gz, gz_gui])