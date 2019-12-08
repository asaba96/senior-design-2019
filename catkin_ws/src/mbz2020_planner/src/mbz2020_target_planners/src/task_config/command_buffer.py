#!/usr/bin/env python

import rospy

from mbz2020_common.msg import (
    MotionTrajectory,
    MotionPointStamped
)

from geometry_msgs.msg import TwistStamped


class CommandBuffer(object):
    def __init__(self):
        self._traj_pub = rospy.Publisher(
            "/motion/trajectory",
            MotionTrajectory,
            queue_size=10,
        )

        self._point_pub = rospy.Publisher(
            "/motion/point",
            MotionPointStamped,
            queue_size=10,
        )

        self._vel_pub = rospy.Publisher(
            "/uav1/velocity_command",
            TwistStamped,
            queue_size=10,
        )

    def publish_trajectory(self, msg):
        self._traj_pub.publish(msg)

    def publish_point(self, motion_point):
        self._point_pub.publish(motion_point)

    def publish_velocity(self, vel):
        vel.header.stamp = rospy.Time.now()
        vel.header.frame_id = '/uav1/base_link_stabilized'
        self._vel_pub.publish(vel)

    def wait_until_ready(self, timeout):
        return
