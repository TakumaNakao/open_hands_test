#!/usr/bin/env python3

import rclpy

rclpy.init_node("pr2_rviz_visualization")


from std_msgs.msg import Float32
from pr2_msgs.msg import BatteryServer

battery_status = {}
battery_pub0 = rclpy.Publisher("/visualization/battery/value0", Float32)
battery_pub1 = rclpy.Publisher("/visualization/battery/value1", Float32)
battery_pub2 = rclpy.Publisher("/visualization/battery/value2", Float32)
battery_pub3 = rclpy.Publisher("/visualization/battery/value3", Float32)
battery_pubs = [battery_pub0,
                battery_pub1,
                battery_pub2,
                battery_pub3]

def batteryCB(msg):
    battery_pubs[msg.id].publish(Float32(msg.averageCharge))

s = rclpy.Subscriber("/battery/server", BatteryServer, batteryCB)

rclpy.spin()


