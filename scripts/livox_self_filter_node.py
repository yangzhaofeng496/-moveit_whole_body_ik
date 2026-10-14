#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class LivoxSelfFilter:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/livox/mid360/points')
        self.output_topic = rospy.get_param('~output_topic', '/livox/mid360/filtered_points')
        self.min_range = rospy.get_param('~min_range', 0.75)
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.callback,
                                    queue_size=1, tcp_nodelay=True)

    def callback(self, msg):
        fields = [f for f in msg.fields if f.name in ('x', 'y', 'z')]
        if len(fields) != 3:
            rospy.logwarn_throttle(5.0, 'Livox cloud has no xyz fields')
            return
        points = []
        min_range_sq = self.min_range * self.min_range
        for x, y, z in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            if x * x + y * y + z * z >= min_range_sq:
                points.append((x, y, z))
        # Publish an empty-but-valid cloud too, so downstream displays never
        # look stale when the scanner is temporarily viewing only the robot.
        out = pc2.create_cloud_xyz32(msg.header, points)
        self.pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('livox_self_filter')
    LivoxSelfFilter()
    rospy.spin()
