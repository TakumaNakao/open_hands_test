#!/usr/bin/env python3

import numpy as np
from scipy.spatial import ConvexHull

import rclpy
from jsk_recognition_msgs.msg import Segment
from jsk_recognition_msgs.msg import SegmentArray
from geometry_msgs.msg import Point


def main():
    rclpy.init_node('segment_array_sample')

    pub = rclpy.Publisher('~output', SegmentArray, queue_size=1)
    r = rclpy.Rate(rclpy.get_param('~rate', 1.0))

    segment_array = SegmentArray()
    segment_array.header.frame_id = rclpy.get_param('~frame_id', 'map')

    N = 30
    while not rclpy.is_shutdown():
        points = np.random.rand(N, 3)
        hull = ConvexHull(points)
        segment_array.header.stamp = rclpy.Time.now()
        segment_array.segments = []
        for i in hull.simplices:
            a, b, c = points[i]
            segment_array.segments.extend(
                [Segment(start_point=Point(a[0], a[1], a[2]),
                         end_point=Point(b[0], b[1], b[2])),
                 Segment(start_point=Point(b[0], b[1], b[2]),
                         end_point=Point(c[0], c[1], c[2])),
                 Segment(start_point=Point(a[0], a[1], a[2]),
                         end_point=Point(c[0], c[1], c[2])),
                ])
        pub.publish(segment_array)
        r.sleep()


if __name__ == '__main__':
    main()
