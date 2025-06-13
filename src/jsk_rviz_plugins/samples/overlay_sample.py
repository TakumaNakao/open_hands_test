#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32
import math
import random


class OverlaySampleNode(Node):
    def __init__(self):
        super().__init__('overlay_sample')
        
        self.text_pub = self.create_publisher(OverlayText, 'text_sample', 1)
        self.value_pub = self.create_publisher(Float32, 'value_sample', 1)
        
        self.counter = 0
        self.rate = 100
        
        # Create timer for publishing at specified rate
        timer_period = 1.0 / self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Overlay sample node started')

    def timer_callback(self):
        self.counter += 1
        
        text = OverlayText()
        theta = self.counter % 255 / 255.0
        text.width = 400
        text.height = 600
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = """This is OverlayText plugin.
The update rate is %d Hz.
You can write several text to show to the operators.
New line is supported and automatical wrapping text is also supported.
And you can choose font, this text is now rendered by '%s'

You can specify background color and foreground color separatelly.

Of course, the text is not needed to be fixed, see the counter: %d.

You can change text color like <span style="color: red;">this</span>
by using <span style="font-style: italic;">css</style>.
        """ % (self.rate, text.font, self.counter)
        
        text.fg_color = ColorRGBA(r=25.0/255.0, g=1.0, b=240.0/255.0, a=1.0)
        text.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.2)
        
        self.text_pub.publish(text)
        
        value_msg = Float32()
        value_msg.data = math.sin(self.counter * math.pi * 2 / 100)
        self.value_pub.publish(value_msg)
        
        # Log messages at different levels
        if int(self.counter % 500) == 0:
            self.get_logger().debug('This is ROS_DEBUG.')
        elif int(self.counter % 500) == 100:
            self.get_logger().info('This is ROS_INFO.')
        elif int(self.counter % 500) == 200:
            self.get_logger().warn('This is ROS_WARN.')
        elif int(self.counter % 500) == 300:
            self.get_logger().error('This is ROS_ERROR.')
        elif int(self.counter % 500) == 400:
            self.get_logger().fatal('This is ROS_FATAL.')


def main(args=None):
    rclpy.init(args=args)
    
    overlay_sample_node = OverlaySampleNode()
    
    try:
        rclpy.spin(overlay_sample_node)
    except KeyboardInterrupt:
        pass
    finally:
        overlay_sample_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

