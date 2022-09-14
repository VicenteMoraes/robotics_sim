import rclpy
import os
from std_msgs.msg import String


def log(msg):
    print(msg.data)


if __name__ == "__main__":
    rclpy.init()
    trial_id = os.environ['TRIAL_ID']
    node = rclpy.create_node(f"logger_{trial_id}")
    sub = node.create_subscription(String, 'log', log, 10)
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
