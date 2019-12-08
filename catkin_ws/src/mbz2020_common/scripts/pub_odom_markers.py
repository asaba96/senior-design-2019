#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PolygonStamped
from visualization_msgs.msg import Marker

def gpscallback(msg):
    marker = Marker()
    marker.header = msg.header
    marker.header.frame_id = "map"
    marker.ns = 'gps'
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.MODIFY

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1

    marker.lifetime = rospy.Duration(2000)
    marker.frame_locked = False

    point = Point()
    point.x = msg.pose.pose.position.x
    point.y = msg.pose.pose.position.y
    point.z = msg.pose.pose.position.z

    gpspoints.append(point)

    marker.points.extend(gpspoints)


    vis_pub.publish(marker)

def odomcallback(msg):
    marker = Marker()
    marker.header = msg.header
    marker.header.frame_id = "map"
    marker.ns = 'camera'
    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.MODIFY

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1

    marker.lifetime = rospy.Duration(2000)
    marker.frame_locked = False

    point = Point()
    point.x = msg.pose.pose.position.x
    point.y = msg.pose.pose.position.y
    point.z = msg.pose.pose.position.z

    campoints.append(point)

    marker.points.extend(campoints)

    cam_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('markers_node')

    vis_pub = rospy.Publisher('/markers',
                              Marker,
                              queue_size=5)

    rospy.Subscriber('odometry/gps', Odometry, gpscallback)

    cam_pub = rospy.Publisher('/markers/cam',
                              Marker,
                              queue_size=5)

    rospy.Subscriber('odometry/filtered', Odometry, odomcallback)

    gpspoints = []
    campoints = []

    rospy.spin()

