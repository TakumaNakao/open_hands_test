#!/usr/bin/env python33

# it depends on jsk_rviz_plugins

import rclpy
from rclpy.node import Node
from jsk_rviz_plugins.msg import OverlayText
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface


class StaticOverlayTextNode(Node):
    def __init__(self):
        super().__init__('static_overlay_text')
        
        # Declare and get parameter
        self.declare_parameter('text', 'Default text')
        self.text = self.get_parameter('text').get_parameter_value().string_value
        
        self.text_interface = OverlayTextInterface(self, 'output')
        self.timer = self.create_timer(0.1, self.publish_text)
        
        self.get_logger().info(f'Static overlay text node started with text: {self.text}')

    def publish_text(self):
        self.text_interface.publish(str(self.text))


def main(args=None):
    rclpy.init(args=args)
    
    node = StaticOverlayTextNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
