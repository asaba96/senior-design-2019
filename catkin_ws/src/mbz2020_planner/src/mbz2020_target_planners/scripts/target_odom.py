#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


if __name__ == "__main__":
    rospy.init_node("target_odom")

    _odom_pub = rospy.Publisher('/target',
                            Odometry,
                            queue_size=10)

    _pub = rospy.Publisher('/markers',
                            Marker,
                            queue_size=5)

    _rate = rospy.Rate(10)

    odom = Odometry()
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = 5
    odom.pose.pose.position.y = 5

    marker = Marker()
    marker.header.frame_id = "world"
    marker.ns = 'target'

    marker.id = 0
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.MODIFY

    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1

    marker.lifetime = rospy.Duration(1000)
    marker.frame_locked = False

    marker.points.append(odom.pose.pose.position)

    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        _odom_pub.publish(odom)
        _pub.publish(marker)
        _rate.sleep()
