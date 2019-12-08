#!/usr/bin/env python
import rospy

from itertools import cycle

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

if __name__ == "__main__":
    rospy.init_node("target_markers")

    _pub = rospy.Publisher('/markers',
                            Marker,
                            queue_size=5)
    _odom_pub = rospy.Publisher('/target',
                            Odometry,
                            queue_size=10)


    _rate = rospy.Rate(10)

    originX = 10
    originY = 10

    markers = []
    odoms = []

    for i in range(0, 50):
        ps = Point()
        ps.x = i * 0.1 + originX
        ps.y = 0.0 + originY
        ps.z = 5.0

        odom = Odometry()
        odom.pose.pose.position = ps
        odom.twist.twist.linear.x = 1.0

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

        marker.points.append(ps)

        markers.append(marker)
        odoms.append(odom)

    for i in range(0, 50):
        ps = Point()
        ps.x = 5.0 + originX
        ps.y = i * 0.1 + originY
        ps.z = 5.0

        odom = Odometry()
        odom.pose.pose.position = ps
        odom.twist.twist.linear.y = 1.0

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

        marker.points.append(ps)

        markers.append(marker)
        odoms.append(odom)

    for i in range(0, 50):
        ps = Point()
        ps.x = 5.0 - i * 0.1 + originX
        ps.y = 5.0 + originY
        ps.z = 5.0

        odom = Odometry()
        odom.pose.pose.position = ps
        odom.twist.twist.linear.x = -1.0

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

        marker.points.append(ps)

        markers.append(marker)
        odoms.append(odom)

    for i in range(0, 50):
        ps = Point()
        ps.x = 0.0 + originX
        ps.y = 5.0 - i * 0.1 + originY
        ps.z = 5.0

        odom = Odometry()
        odom.pose.pose.position = ps
        odom.twist.twist.linear.y = -1.0

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

        marker.points.append(ps)

        markers.append(marker)
        odoms.append(odom)

    marker_pool = cycle(zip(markers, odoms))

    while not rospy.is_shutdown():
        for mark, odom in marker_pool:
            if rospy.is_shutdown():
                break
            mark.header.stamp = rospy.Time.now()
            odom.header.stamp = rospy.Time.now()
            _pub.publish(mark)
            _odom_pub.publish(odom)
            _rate.sleep()
