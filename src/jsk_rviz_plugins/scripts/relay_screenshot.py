#!/usr/bin/env python3
"""
Call snapshot service of rviz (provided by ScreenshotListener tool)
when a topic is published.

This script is useful to automatically record result of ros processing.

NOTE:
  rviz should be in fron of other windows because 
"""

import rclpy
from jsk_rviz_plugins.srv import Screenshot

def callback(msg):
    global counter
    rclpy.loginfo('received a message, save a screenshot to {0}'.format(file_format.format(counter)))
    try:
        screenshot_srv(file_format.format(counter))
        counter = counter + 1
    except rclpy.ServiceException as e:
        rclpy.logerr('Failed to call screenshot service call. Have you add ScreenshotListener to rviz and file_format is correct? file_format is "{0}"'.format(file_format))
        
    
if __name__ == '__main__':
    counter = 0
    rclpy.init_node('relay_screenshot')
    screenshot_srv = rclpy.ServiceProxy('/rviz/screenshot', Screenshot)
    file_format = rclpy.get_param('~file_format', 'rviz_screenshot_{0:0>5}.png')
    sub = rclpy.Subscriber('~input', rclpy.msg.AnyMsg, callback)
    rclpy.spin()
