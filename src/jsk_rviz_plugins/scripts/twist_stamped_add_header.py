#!/usr/bin/env python3

import rclpy
import sys
from geometry_msgs.msg import Twist, TwistStamped

rclpy.init_node("twist_stamped_add_header")
pub = rclpy.Publisher("cmd_vel_stamped", TwistStamped)

def callback(msg):
    global pub
    output = TwistStamped()
    output.header.stamp = rclpy.Time.now()
    output.header.frame_id = sys.argv[1]
    output.twist = msg
    pub.publish(output)

if len(sys.argv) != 3:
    print("Usage: twist_stamped_add_header frame_id topic")

sub = rclpy.Subscriber(sys.argv[2], Twist, callback)
rclpy.spin()

