#!/usr/bin/env python33

import re
import rclpy
from rclpy.node import Node
from jsk_rviz_plugins.msg import OverlayText
from rcl_interfaces.msg import Log
from std_msgs.msg import ColorRGBA


class RosConsoleOverlayTextNode(Node):
    def __init__(self):
        super().__init__('rosconsole_overlay_text')
        
        # Declare parameters
        self.declare_parameter('nodes', [])
        self.declare_parameter('nodes_regexp', '')
        self.declare_parameter('ignore_nodes', [])
        self.declare_parameter('exclude_regexes', [])
        self.declare_parameter('line_buffer_length', 100)
        self.declare_parameter('reverse_lines', True)
        
        # Get parameters
        self.nodes = self.get_parameter('nodes').get_parameter_value().string_array_value
        self.nodes_regexp = self.get_parameter('nodes_regexp').get_parameter_value().string_value
        self.ignore_nodes = self.get_parameter('ignore_nodes').get_parameter_value().string_array_value
        self.exclude_regexes = self.get_parameter('exclude_regexes').get_parameter_value().string_array_value
        self.line_buffer_length = self.get_parameter('line_buffer_length').get_parameter_value().integer_value
        self.reverse_lines = self.get_parameter('reverse_lines').get_parameter_value().bool_value
        
        if self.nodes_regexp:
            self.nodes_regexp_compiled = re.compile(self.nodes_regexp)
        else:
            self.nodes_regexp_compiled = None
            
        self.lines = []
        
        # Create subscriber and publisher
        self.sub = self.create_subscription(Log, '/rosout', self.callback, 10)
        self.pub = self.create_publisher(OverlayText, 'output', 1)
        
        self.get_logger().info('RosConsole overlay text node started')

    def colored_message(self, msg):
        cmsg = msg.msg
        cmsg = re.sub(r'\x1b\[31m', '<span style="color: red">', cmsg)
        cmsg = re.sub(r'\x1b\[32m', '<span style="color: green">', cmsg)
        cmsg = re.sub(r'\x1b\[33m', '<span style="color: yellow">', cmsg)
        cmsg = re.sub(r'\x1b\[34m', '<span style="color: blue">', cmsg)
        cmsg = re.sub(r'\x1b\[35m', '<span style="color: purple">', cmsg)
        cmsg = re.sub(r'\x1b\[36m', '<span style="color: cyan">', cmsg)
        cmsg = re.sub(r'\x1b\[0m', '</span>', cmsg)
        
        if msg.level == Log.DEBUG:
            return '<span style="color: rgb(120,120,120);">%s</span>' % cmsg
        elif msg.level == Log.INFO:
            return '<span style="color: white;">%s</span>' % cmsg
        elif msg.level == Log.WARN:
            return '<span style="color: yellow;">%s</span>' % cmsg
        elif msg.level == Log.ERROR:
            return '<span style="color: red;">%s</span>' % cmsg
        elif msg.level == Log.FATAL:
            return '<span style="color: red;">%s</span>' % cmsg
        return cmsg

    def callback(self, msg):
        for exclude_regex in self.exclude_regexes:
            if re.match(exclude_regex, msg.msg):
                return

        if msg.name not in self.ignore_nodes:
            if msg.name in self.nodes or len(self.nodes) == 0:
                if (len(self.nodes_regexp) == 0 or 
                    (self.nodes_regexp_compiled and self.nodes_regexp_compiled.match(msg.name))):
                    
                    if self.reverse_lines:
                        self.lines = [self.colored_message(msg)] + self.lines
                        if len(self.lines) > self.line_buffer_length:
                            self.lines = self.lines[0:self.line_buffer_length]
                    else:
                        self.lines = self.lines + [self.colored_message(msg)]
                        if len(self.lines) > self.line_buffer_length:
                            self.lines = self.lines[-self.line_buffer_length:]
                    
                    text = OverlayText()
                    text.left = 20
                    text.top = 20
                    text.width = 1200
                    text.height = 1200
                    text.fg_color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=1.0)
                    text.text_size = 12
                    text.text = "\n".join(self.lines)
                    self.pub.publish(text)


def main(args=None):
    rclpy.init(args=args)
    
    node = RosConsoleOverlayTextNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
