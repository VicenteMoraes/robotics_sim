# Adapted from https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/tb3_simulation_launch.py
import os
import json

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    log_settings = LaunchConfiguration('log_settings', default='true')

    namespace = os.environ['ROBOT_NAMESPACE']
    pose = json.loads(os.environ['ROBOT_POSE'])
    use_rviz = os.environ['USE_RVIZ']
    rviz_config_file = LaunchConfiguration('rviz_config')

    params_file = os.environ['PARAMS_YAML'] if os.environ['PARAMS_YAML'] else '/workdir/param/waffle.yaml'
    map_yaml_file = os.environ['MAP_YAML'] if os.environ['MAP_YAML'] else os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    nav2_robot = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rviz_launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': TextSubstitution(text=namespace),
                              'use_namespace': 'True',
                              'rviz_config': rviz_config_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                       'launch',
                                                       'tb3_simulation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_namespace': 'True',
                              'map': map_yaml_file,
                              'use_sim_time': 'True',
                              'params_file': params_file,
                              'autostart': 'True',
                              'use_rviz': 'False',
                              'use_simulator': 'False',
                              'headless': 'False',
                              'use_robot_state_pub': 'True',
                              'use_composition': 'False',
                              'x_pose': TextSubstitution(text=str(pose['x'])),
                              'y_pose': TextSubstitution(text=str(pose['y'])),
                              'z_pose': TextSubstitution(text=str(pose['z'])),
                              'roll': TextSubstitution(text=str(pose['roll'])),
                              'pitch': TextSubstitution(text=str(pose['pitch'])),
                              'yaw': TextSubstitution(text=str(pose['yaw'])),
                              'robot_name': TextSubstitution(text=namespace), }.items()),

        LogInfo(
            condition=IfCondition(log_settings),
            msg=['Launching ', namespace]),
        LogInfo(
            condition=IfCondition(log_settings),
            msg=[namespace, ' map yaml: ', map_yaml_file]),
        LogInfo(
            condition=IfCondition(log_settings),
            msg=[namespace, ' params yaml: ', params_file]),
        LogInfo(
            condition=IfCondition(log_settings),
            msg=[namespace, ' using robot state pub: ', 'True']),
        LogInfo(
            condition=IfCondition(log_settings),
            msg=[namespace, ' autostart: ', 'True'])
    ])

    ld = LaunchDescription()
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(nav2_robot)
    return ld

