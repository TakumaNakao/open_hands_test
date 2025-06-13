#!/usr/bin/env python3

import rclpy
from hrpsys_ros_bridge.msg import ContactState, ContactStateStamped, ContactStatesStamped
from random import random
if __name__ == "__main__":
    rclpy.init_node("contact_state_sample")
    pub = rclpy.Publisher("~output", ContactStatesStamped)
    link_names = rclpy.get_param("~links", ["r_shoulder_pan_link"])
    rate = rclpy.Rate(1)
    while not rclpy.is_shutdown():
        states = ContactStatesStamped()
        states.header.stamp = rclpy.Time.now()
        for link_name in link_names:
            state = ContactStateStamped()
            state.header.frame_id = link_name
            state.header.stamp = rclpy.Time.now()
            if random() < 0.5:
                state.state.state = ContactState.ON
            else:
                state.state.state = ContactState.OFF
            states.states.append(state)
        pub.publish(states)
        rate.sleep()
