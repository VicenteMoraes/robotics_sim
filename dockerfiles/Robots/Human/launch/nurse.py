import json
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time


def formatlog(severity, who, loginfo, skill, params):
    global simulation_init_time
    return ('[' + severity + '],' +
            who + ',' +
            loginfo + ',' +
            skill + ',' +
            params)


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
        content = {
            'skill': 'wait-message',
            'status': 'message-received'
        }
        logdata = {
            'level': 'info',
            'entity': 'nurse',
            'content': content
        }
        log.data = json.dumps(logdata)
        print("NURSE")
        print(log.data)
        print(com_data.data)
        print("NURSE")
        self.pub_log.publish(log)
        # if com_data.data == "Open Drawer":
        print(pub_str.data)
        rate = self.create_rate(0.5)
        for i in range(0, 5):
            #rospy.loginfo(pub_str)
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
        content = {
            'skill': 'authentication-request',
            'status': 'message-received'
        }
        logdata = {
            'level': 'info',
            'entity': 'nurse',
            'content': content
        }
        log.data = json.dumps(logdata)
        self.pub_log.publish(log)

        self.pub.publish(pub_str)
        log.data = self.name + ": sent " + str(pub_str)
        self.pub_log.publish(log)

        log.data = formatlog('info',
                             self.name,
                             'sync',
                             'request-sent',
                             '(status=waiting)')
        content = {
            'skill': 'authentication-sent',
            'status': 'waiting'
        }
        logdata = {
            'level': 'info',
            'entity': 'nurse',
            'content': content
        }
        log.data = json.dumps(logdata)
        self.pub_log.publish(log)


if __name__ == "__main__":
    rclpy.init()
    nurse = Nurse()
    rclpy.spin(nurse)