#!/usr/bin/env python3

from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA
import rclpy
from rclpy.node import Node


class OverlayTextInterface():
    def __init__(self, node, topic):
        self.node = node
        self.pub = node.create_publisher(OverlayText, topic, 1)
        
        # Declare parameters with default values
        self.node.declare_parameter('width', 400)
        self.node.declare_parameter('height', 600)
        self.node.declare_parameter('top', 10)
        self.node.declare_parameter('left', 10)
        self.node.declare_parameter('fg_alpha', 1.0)
        self.node.declare_parameter('fg_red', 1.0)
        self.node.declare_parameter('fg_green', 1.0)
        self.node.declare_parameter('fg_blue', 1.0)
        self.node.declare_parameter('bg_alpha', 0.0)
        self.node.declare_parameter('bg_red', 0.0)
        self.node.declare_parameter('bg_green', 0.0)
        self.node.declare_parameter('bg_blue', 0.0)
        self.node.declare_parameter('text_size', 12)
        
    def get_config(self):
        """Get current parameter values"""
        config = {}
        config['width'] = self.node.get_parameter('width').get_parameter_value().integer_value
        config['height'] = self.node.get_parameter('height').get_parameter_value().integer_value
        config['top'] = self.node.get_parameter('top').get_parameter_value().integer_value
        config['left'] = self.node.get_parameter('left').get_parameter_value().integer_value
        config['fg_alpha'] = self.node.get_parameter('fg_alpha').get_parameter_value().double_value
        config['fg_red'] = self.node.get_parameter('fg_red').get_parameter_value().double_value
        config['fg_green'] = self.node.get_parameter('fg_green').get_parameter_value().double_value
        config['fg_blue'] = self.node.get_parameter('fg_blue').get_parameter_value().double_value
        config['bg_alpha'] = self.node.get_parameter('bg_alpha').get_parameter_value().double_value
        config['bg_red'] = self.node.get_parameter('bg_red').get_parameter_value().double_value
        config['bg_green'] = self.node.get_parameter('bg_green').get_parameter_value().double_value
        config['bg_blue'] = self.node.get_parameter('bg_blue').get_parameter_value().double_value
        config['text_size'] = self.node.get_parameter('text_size').get_parameter_value().integer_value
        return config
        
    def publish(self, text):
        config = self.get_config()
        
        msg = OverlayText()
        msg.text = text
        msg.width = config['width']
        msg.height = config['height']
        msg.top = config['top']
        msg.left = config['left']
        
        msg.fg_color = ColorRGBA()
        msg.fg_color.a = config['fg_alpha']
        msg.fg_color.r = config['fg_red']
        msg.fg_color.g = config['fg_green']
        msg.fg_color.b = config['fg_blue']
        
        msg.bg_color = ColorRGBA()
        msg.bg_color.a = config['bg_alpha']
        msg.bg_color.r = config['bg_red']
        msg.bg_color.g = config['bg_green']
        msg.bg_color.b = config['bg_blue']
        
        msg.text_size = config['text_size']
        self.pub.publish(msg)
