import os
import json
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    pose = json.loads(os.environ['ROBOT_POSE'])

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', os.environ['ROBOT_NAME'],
            '-file', os.environ['ROBOT_SDF'],
            #'-robot_namespace', os.environ['ROBOT_NAMESPACE'],
            '-x', str(pose['x']), '-y', str(pose['y']), '-z', str(pose['z']),
            '-R', str(pose['roll']), '-P', str(pose['pitch']), '-Y', str(pose['yaw'])])

    ld = LaunchDescription()
    ld.add_action(start_gazebo_spawner_cmd)
    return ld