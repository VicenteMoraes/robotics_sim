import rclpy
import os
from std_msgs.msg import String

node = None


def log(msg):
    global node
    now = node.get_clock().now().to_msg()
    print(str(now.sec) + ' ,' + msg.data)


if __name__ == "__main__":
    rclpy.init()
    trial_id = os.environ['TRIAL_ID']
    node = rclpy.create_node(f"logger_{trial_id}")
    sub = node.create_subscription(String, 'log', log, 10)
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
