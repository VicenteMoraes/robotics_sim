import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler
import json


ROBOT_NAME = os.environ['ROBOT_NAME']
ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']
CONFIG = json.loads(os.environ['CONFIG'])


def formatlog(severity, who, loginfo, variable=None, skill=None, params=None):
    return f"[{severity}], {who}, {loginfo}, {variable}, {skill}, {params}"


class Logger(Node):
    def __init__(self, use_sim_time=True):
        use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        super(Logger, self).__init__(f"{ROBOT_NAME}_logger", parameter_overrides=[use_sim_time])

        self.name = ROBOT_NAME
        self.namespace = ROBOT_NAMESPACE
        self.config = CONFIG

        self.pose = None
        self.odom = None
        self.twist = None

        self.odom_sub = self.create_subscription(Odometry, f"/{self.namespace}/odom", self.update_odom, 10)
        self.log_pub = self.create_publisher(String, "/log", 5)

        self.init_timer = self.create_timer(0.1, self.init_log)
        self.update_timer = self.create_timer(10, self.update_log)

    def update_odom(self, odom):
        self.odom = odom
        self.pose = odom.pose.pose
        self.twist = odom.twist

    def init_log(self):
        msg = String()
        msg.data = formatlog("DEBUG", "logger", f"ROBOTS_CONFIG={self.config}")
        self.log_pub.publish(msg)

        msg.data = formatlog("DEBUG", "logger", "Simulation open")
        self.log_pub.publish(msg)
        self.init_timer.destroy()

    def update_log(self):
        try:
            orientation = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
            pose_msg = String()
            pose_msg.data = formatlog("INFO", self.name, {'y': round(self.pose.position.y, 3), 'x': round(self.pose.position.x, 3),
                                                          "yaw": round(quat2euler(orientation)[2], 3)}, variable="pose")
            self.log_pub.publish(pose_msg)
        except AttributeError:
            pass


if __name__ == "__main__":
    rclpy.init()
    node = Logger()
    rclpy.spin(node)