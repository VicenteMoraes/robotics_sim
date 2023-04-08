import rclpy
from rclpy.node import Node
import os
import json
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist


def formatlog(severity, who, loginfo: dict, skill=None, params=None):
    return f"[{severity}], {who}, {loginfo}, {skill}, {params}"

class BatterySensor(Node):
    def __init__(self, parent,
            capacity=1800,
            initial_percentage=1,
            discharge_rate_percentage=0.0005,
            discharge_rate_ah=0):
        super(BatterySensor, self).__init__('battery_sensor')
        self.parent = parent
        self.capacity = 1800.0
        self.percentage = initial_percentage
        self.voltage = 11.1
        self.status = 0
        self.health = 0
        self.tech = 3
        self.current = 0.8
        self.design_capacity = 1800.0
        self.discharge_rate_ah = discharge_rate_ah
        self.discharge_rate_percentage = discharge_rate_percentage
        self.namespace = os.environ['ROBOT_NAMESPACE']

        self.charge = capacity*initial_percentage

        self.battery_pub = self.create_publisher(BatteryState, f"/{self.namespace}/battery",  1)
        self.log_pub = self.create_publisher(String, "/log",  1)
        self.vel_pub = self.create_publisher(Twist, f"/{self.namespace}/cmd_vel",  1)

        self.update_timer = self.create_timer(1, self.update_charge)
        self.log_timer = self.create_timer(15, self.update_log)
        self.update_log()

    def update_log(self):
        log = String()
        log.data = formatlog("INFO", self.parent, {'battery-level': f"{self.percentage*100:02.2f}"})
        self.log_pub.publish(log)

    def update_charge(self):
        msg = BatteryState()
        msg.voltage = self.voltage
        msg.current = self.current
        msg.capacity = self.capacity
        msg.design_capacity = self.design_capacity
        msg.power_supply_status = self.status
        msg.power_supply_health = self.health
        msg.power_supply_technology = self.tech
        msg.present = True

        msg.charge = self.charge - self.discharge_rate_ah if self.discharge_rate_ah != 0 else self.charge - self.discharge_rate_percentage * self.capacity
        msg.percentage = self.charge/self.capacity

        self.battery_pub.publish(msg)
        self.charge = msg.charge
        self.percentage = msg.percentage
        if msg.percentage < .05:
            #rospy.logwarn(f"{self.parent} requesting simulation end...")
            log = String()
            log.data = "ENDLOWBATT"
            self.log_pub.publish(log)
            #self.new_timer = rospy.Timer(rospy.Duration(1.0/100.0), self.stop_robot)
            self.stop_robot()
            # self.timer.cancel()
            #self.timer.shutdown()
            # return

    def stop_robot(self):
        vel_0 = Twist()
        self.vel_pub.publish(vel_0)
        self.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parent = os.environ['ROBOT_NAME']
    rclpy.init()
    try:
        config = json.loads(os.environ['CONFIG'])
        initial_percentage = config['battery_charge']
        discharge_rate_percentage = config['battery_discharge_rate']
        battery = BatterySensor(parent, initial_percentage=initial_percentage,
                                discharge_rate_percentage=discharge_rate_percentage)
    except (AttributeError, json.decoder.JSONDecodeError):
        battery = BatterySensor(parent)

    rclpy.spin(battery)
