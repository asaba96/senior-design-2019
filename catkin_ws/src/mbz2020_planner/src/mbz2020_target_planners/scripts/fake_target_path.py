#!/usr/bin/env python
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("fake_target_path")

    _pub = rospy.Publisher("fake_target", Path, queue_size=10)
    _rate = rospy.Rate(10)

    originX = 10
    originY = 10

    p = Path()
    p.header.frame_id = "world"

    for i in range(0, 50):
        ps = PoseStamped()
        ps.pose.position.x = i * 0.1 + originX
        ps.pose.position.y = 0.0 + originY
        ps.pose.position.z = 5.0

        p.poses.append(ps)

    for i in range(0, 50):
        ps = PoseStamped()
        ps.pose.position.x = 5.0 + originX
        ps.pose.position.y = i * 0.1 + originY
        ps.pose.position.z = 5.0

        p.poses.append(ps)
    for i in range(0, 50):
        ps = PoseStamped()
        ps.pose.position.x = 5.0 - i * 0.1 + originX
        ps.pose.position.y = 5.0 + originY
        ps.pose.position.z = 5.0

        p.poses.append(ps)

    for i in range(0, 50):
        ps = PoseStamped()
        ps.pose.position.x = 0.0 + originX
        ps.pose.position.y = 5.0 - i * 0.1 + originY
        ps.pose.position.z = 5.0

        p.poses.append(ps)

    while not rospy.is_shutdown():
        p.header.stamp = rospy.Time.now()
        _pub.publish(p)
        _rate.sleep()
