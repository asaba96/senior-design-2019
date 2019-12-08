#!/usr/bin/env python

import rospy
import threading
import math
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu


class OdometryFilter(object):
    def __init__(self):
        self._lock = threading.Lock()

        try:
            _topic = 'mavros/imu/data/unfixed'
        except KeyError:
            rospy.logerr("ImuFilter: unable to lookup parameter")
            raise

        self._odom_pub = rospy.Publisher(
            "mavros/imu/data", Imu, queue_size=10
        )

        self._sub = rospy.Subscriber(_topic, Imu, self._odom_callback)

    def _odom_callback(self, msg):
        with self._lock:
            _time = msg.header.stamp

            msg_fixed = msg

            # I have zero clue why I need to do this
            msg_fixed.angular_velocity_covariance = list(msg_fixed.angular_velocity_covariance)
            msg_fixed.linear_acceleration_covariance = list(msg_fixed.linear_acceleration_covariance)

            msg_fixed.angular_velocity_covariance[0] = 0.024674
            msg_fixed.angular_velocity_covariance[4] = 0.024674
            msg_fixed.angular_velocity_covariance[8] = 0.024674

            msg_fixed.linear_acceleration_covariance[0] = 0.0225
            msg_fixed.linear_acceleration_covariance[4] = 0.0225
            msg_fixed.linear_acceleration_covariance[8] = 0.0225

            self._odom_pub.publish(msg_fixed)

if __name__ == "__main__":
    rospy.init_node("imu_filter")
    node_ = OdometryFilter()
    rospy.spin()
