#!/usr/bin/env python3

"""
Publish a visualization_marker for specified link
"""

import rclpy
from visualization_msgs.msg import Marker
from xml.dom.minidom import parse, parseString

if __name__ == "__main__":
    rclpy.init_node("link_marker_publisher")
    link_name = rclpy.get_param("~link")
    rgb = rclpy.get_param("~rgb", [1, 0, 0])
    alpha = rclpy.get_param("~alpha", 1.0)
    scale = rclpy.get_param("~scale", 1.02)
    robot_description = rclpy.get_param("/robot_description")
    # Parse robot_description using minidom directly
    # because urdf_parser_py cannot read PR2 urdf
    doc = parseString(robot_description)
    links = doc.getElementsByTagName('link')
    mesh_file = None
    for link in links:
        if link_name == link.getAttribute('name'):
            visual_mesh = link.getElementsByTagName('visual').item(0).getElementsByTagName('mesh').item(0)
            mesh_file = visual_mesh.getAttribute('filename')
            break
    if not mesh_file:
        raise Exception("Cannot find link: {0}".format(link_name))
    pub = rclpy.Publisher('~marker', Marker)
    rate = rclpy.Rate(1)
    while not rclpy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = link_name
        marker.header.stamp = rclpy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        marker.color.a = alpha
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.mesh_resource = mesh_file
        marker.frame_locked = True
        pub.publish(marker)
        rate.sleep()
