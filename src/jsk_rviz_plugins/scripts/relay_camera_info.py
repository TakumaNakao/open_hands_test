#!/usr/bin/env python3
import rclpy
from sensor_msgs.msg import CameraInfo

class RelayCameraInfo():
    def __init__(self):
        rclpy.init_node('relay_camera_info')
        self.frame_id = rclpy.get_param('~frame_id')
        self.pub = rclpy.Publisher("output", CameraInfo)
        rclpy.Subscriber("input", CameraInfo, self.callback)
        rclpy.spin()

    def callback(self, info):
        info.header.frame_id = self.frame_id
        self.pub.publish(info)

if __name__ == '__main__':
    RelayCameraInfo()
