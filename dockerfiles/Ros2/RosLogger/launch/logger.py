import rclpy
from rclpy.parameter import Parameter
import os
from std_msgs.msg import String

node = None
initial_time = 0


def log(msg):
    global node, initial_time
    now = node.get_clock().now().to_msg().sec + node.get_clock().now().to_msg().nanosec * 10e-10
    print(f"{now:.2f}" + ', ' + msg.data)


if __name__ == "__main__":
    rclpy.init()
    trial_id = os.environ['TRIAL_ID']
    use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, True)
    node = rclpy.create_node(f"logger_{trial_id}", parameter_overrides=[use_sim_time])
    sub = node.create_subscription(String, 'log', log, 10)
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
