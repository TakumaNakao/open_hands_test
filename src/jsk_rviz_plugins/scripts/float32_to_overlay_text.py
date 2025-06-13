#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Lock
from jsk_rviz_plugins.overlay_text_interface import OverlayTextInterface


class Float32ToOverlayTextNode(Node):
    def __init__(self):
        super().__init__('float32_to_overlay_text')
        
        # Declare parameters
        self.declare_parameter('multi_topics', [])
        self.declare_parameter('format', 'value: {0}')
        
        # Get parameters
        self.multi_topics = self.get_parameter('multi_topics').get_parameter_value().string_array_value
        self.g_format = self.get_parameter('format').get_parameter_value().string_value
        
        # Initialize
        self.g_lock = Lock()
        self.g_msg = None
        self.multi_topic_msgs = {}
        
        self.text_interface = OverlayTextInterface(self, 'text')
        
        if self.multi_topics:
            self.subs = []
            for topic in self.multi_topics:
                callback_obj = MultiTopicCallback(topic, self)
                sub = self.create_subscription(Float32, topic, callback_obj.callback, 10)
                self.subs.append(sub)
            self.timer = self.create_timer(0.1, self.publish_text_multi)
        else:
            self.sub = self.create_subscription(Float32, 'input', self.callback, 10)
            self.timer = self.create_timer(0.1, self.publish_text)
        
        self.get_logger().info('Float32 to overlay text node started')

    def callback(self, msg):
        with self.g_lock:
            self.g_msg = msg

    def publish_text(self):
        with self.g_lock:
            if not self.g_msg:
                return
            self.text_interface.publish(self.g_format.format(self.g_msg.data))

    def publish_text_multi(self):
        with self.g_lock:
            if all([msg for topic, msg in self.multi_topic_msgs.items()]):
                total = sum([msg.data for topic, msg in self.multi_topic_msgs.items()])
                self.text_interface.publish(self.g_format.format(total))


class MultiTopicCallback:
    def __init__(self, topic, node):
        self.topic = topic
        self.node = node
        node.multi_topic_msgs[self.topic] = None
        
    def callback(self, msg):
        with self.node.g_lock:
            self.node.multi_topic_msgs[self.topic] = msg


def main(args=None):
    rclpy.init(args=args)
    
    node = Float32ToOverlayTextNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
