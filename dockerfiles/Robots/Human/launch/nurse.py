import json
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time


def formatlog(severity, who, loginfo, skill=None, params=None):
    return f"[{severity}], {who}, {loginfo}, {skill}, {params}"


class Nurse(Node):
    def __init__(self, name: str = 'nurse', path='human_rig'):
        super(Nurse, self).__init__("nurse_node")
        self.name = name
        self.path = path
        self.pub = self.create_publisher(String, f"{name}/fauth",  5)
        self.pub_action = self.create_publisher(String, f"{name}/action",  5)
        self.pub_log = self.create_publisher(String, "log",  5)
        self.pub_comms = self.create_publisher(String, "nurse/comms",  5)
        self.pub_dum2 = self.create_publisher(String, "/led_strip/display",  5)

        self.sub = self.create_subscription(String, "/led_strip/display", self.handle_auth, 10)
        self.sub_comms = self.create_subscription(String, "/nurse/comms", self.comms, 10)

    def comms(self, com_data):
        pub_str = String()
        pub_str.data = "r1"
        if com_data.data == pub_str.data:
            return
        # self.sub_comms_new = rospy.Subscriber(f"{self.name}/comms", String, self.placeholder)
        #rospy.logwarn(com_data)
        log = String()
        log.data = formatlog('info',
                             self.name,
                             'sync',
                             'wait-message',
                             '(status=message-received)')
        self.pub_log.publish(log)
        rate = self.create_rate(0.5)
        for i in range(0, 5):
            self.pub_comms.publish(pub_str)
            rate.sleep()

    def handle_auth(self, msg):
        pub_str = String()
        log = String()
        pub_str.data = "auth"
        # log.data = self.name + ": athentication received "+str(pub_str)
        log.data = formatlog('info',
                             self.name,
                             'sync',
                             'received-request',
                             '(status=sending-request)')
        self.pub_log.publish(log)
        self.pub.publish(pub_str)

        log.data = formatlog('info',
                             self.name,
                             'sync',
                             'request-sent',
                             '(status=waiting)')
        self.pub_log.publish(log)


if __name__ == "__main__":
    rclpy.init()
    nurse = Nurse()
    rclpy.spin(nurse)