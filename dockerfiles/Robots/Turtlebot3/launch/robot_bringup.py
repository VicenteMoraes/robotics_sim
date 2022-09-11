import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as et


def main():
    # Get input arguments from user
    argv = sys.argv[1:]

    namespace = argv[1]
    # Start node
    rclpy.init()
    #sdf_file_path = os.path.join(
    #    get_package_share_directory("turtlebot3_gazebo"), "models",
    #    "turtlebot3_burger", "model-1_4.sdf")
    node = rclpy.create_node("entity_spawner")

    client = node.create_client(SpawnEntity, "/spawn_entity")

    if not client.service_is_ready():
        client.wait_for_service()

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = '/workdir/launch/model-1_4.sdf'

    xml_tree = et.parse(sdf_file_path)
    root = xml_tree.getroot()
    model_root = root.find("model")
    odom = [_ for _ in model_root.iter("odometry_frame")][0]
    odom.text = f"{namespace}/odom"
    base_footprint = [_ for _ in model_root.iter("robot_base_frame")][0]
    base_footprint.text = f"{namespace}/base_footprint"
    xml_str = et.tostring(root).decode()
    xml_str = '<?xml version = "1.0" ?>\n' + xml_str

    # Set data for request
    request = SpawnEntity.Request()
    request.name = argv[0]
    request.xml = xml_str #open(sdf_file_path, 'r').read()
    request.robot_namespace = '/' + argv[1]
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])
    request.initial_pose.orientation.x = float(argv[5])
    request.initial_pose.orientation.y = float(argv[6])
    request.initial_pose.orientation.z = float(argv[7])
    request.initial_pose.orientation.w = float(argv[8])

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()