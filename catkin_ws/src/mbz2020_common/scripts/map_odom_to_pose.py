#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class MapOdomToPose(object):
    def __init__(self):
        self._pose = PoseWithCovarianceStamped()
        self._sub = rospy.Subscriber('/odometry/filtered', Odometry, self._odom_callback)
        self._pub = rospy.Publisher('/mavros/vision_pose/pose_cov', PoseWithCovarianceStamped, queue_size=10)

    def _odom_callback(self, msg):
        self._pose.header = msg.header
        self._pose.pose = msg.pose
        self._pub.publish(self._pose)

if __name__ == '__main__':
    rospy.init_node('map_odom_to_pose')
    node_ = MapOdomToPose()
    rospy.spin()
