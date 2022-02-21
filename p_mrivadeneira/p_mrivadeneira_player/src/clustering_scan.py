#!/usr/bin/env python3
import random

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from math import *
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import math
import random

publisher = rospy.Publisher('/markers', MarkerArray, queue_size=1)
name = rospy.get_name()
final_name = ''

if name.find('red') >= 0:
    final_name = 'red' + name[4]
elif name.find('green') >= 0:
    final_name = 'green' + name[6]
elif name.find('blue') >= 0:
    final_name = 'blue' + name[5]

def CreateMarker():
    # Create marker
    marker = Marker()
    marker.header.frame_id = final_name + "/base_footprint"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "my_namespace"
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.pose.orientation.w = 1.0  # Normalize quaternion
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 0.5
    marker.color.r = random.random()
    marker.color.g = random.random()
    marker.color.b = random.random()

    return marker

def callbackMessageReceived(msg):
    # rospy.loginfo('Received Laser Scan message')

    x_prev, y_prev = 1000, 1000
    dist_threshold = 1.5

    marker_array = MarkerArray()

    # z = 0.14
    z = 0.07        # In order tp get a better "real" position when transforming the marker to a camera pixel

    for idx, range in enumerate(msg.ranges):

        if range < 0.1 or range == inf:         # Take out all Infinite Markers or non reflected laser
            continue

        theta = msg.angle_min + msg.angle_increment*idx
        x = range * cos(theta)
        y = range * sin(theta)


        # Should I create a new cluster?
        dist = math.sqrt((x_prev-x)**2 + (y_prev-y)**2)
        if dist > dist_threshold:       # New cluster
            marker = CreateMarker()
            idx_group = len(marker_array.markers)
            marker.id = idx_group
            marker.points = []
            marker_array.markers.append(marker)

        if len(marker_array.markers) > 0:
            last_marker = marker_array.markers[-1]
            # last_marker.points.append(Point(x=x,y=y-0.07,z=0.07))          # Transformation from base_scan to camera_rgb_frame ==> x=-0.029; y=0, z=0.14
            last_marker.points.append(Point(x=x,y=y,z=z))


        x_prev = x
        y_prev = y
        
    # marker_in_camera = tf_buffer.transform(marker_array, final_name + '/base_footprint', rospy.Duration(1))

    publisher.publish(marker_array)
    # rospy.loginfo('Publishing Marker Array')


def main():

    rospy.init_node('lidar_subscriber', anonymous=False)

    rospy.Subscriber("/scan", LaserScan, callbackMessageReceived)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
