import json
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time
import os
from std_msgs.msg import String


CONFIG = json.loads(os.environ['CONFIG'])


def formatlog(severity, who, loginfo, skill=None, params=None):
    msg = String()
    msg.data = f"[{severity}], {who}, {loginfo}, {skill}, {params}"
    return msg


class Nurse(Node):
    def __init__(self, name: str = 'nurse', path='human_rig'):
        super(Nurse, self).__init__("nurse_node")
        self.name = name
        self.path = path
        self.pub = self.create_publisher(String, f"{name}/fauth",  5)
        self.pub_action = self.create_publisher(String, f"{name}/action",  5)
        self.pub_log = self.create_publisher(String, "log",  5)
        self.pub_comms = self.create_publisher(String, "/nurse/comms",  5)
        self.arm_comms = self.create_publisher(String, "/lab_arm/comms",  5)
        self.pub_dum2 = self.create_publisher(String, "/led_strip/display",  5)

        self.sub = self.create_subscription(String, "/led_strip/display", self.handle_auth, 10)
        self.arm_sub = self.create_subscription(String, "/lab_arm/comms", self.comms, 10)
        self.sub_comms = self.create_subscription(String, "/nurse/comms", self.comms, 10)

    def comms(self, com_data):
        pub_str = String()
        pub_str.data = "r1"
        if com_data.data == pub_str.data:
            return
        log = formatlog('info',
                        com_data.data,
                        'sync',
                        'wait-message',
                        '(status=message-received)')
        self.pub_log.publish(log)
        time.sleep(1)
        if com_data.data == 'nurse':
            self.pub_comms.publish(pub_str)
        elif com_data.data == 'lab_arm':
            self.arm_comms.publish(pub_str)

    def handle_auth(self, msg):
        pub_str = String()
        pub_str.data = "auth"
        log = formatlog('info',
                        self.name,
                        'sync',
                        'received-request',
                        '(status=sending-request)')
        self.pub_log.publish(log)
        self.pub.publish(pub_str)

        log = formatlog('info',
                        self.name,
                        'sync',
                        'request-sent',
                        '(status=waiting)')
        self.pub_log.publish(log)


if __name__ == "__main__":
    rclpy.init()
    print('Started Nurse')
    nurse = Nurse()
    time.sleep(1)
    nurse.pub_log.publish(formatlog('DEBUG', 'nurse', f'NURSE_CONFIG={CONFIG}'))
    print("Published Log")
    print("\n Starting Nurse System")
    rclpy.spin(nurse)