#!/usr/bin/env python

import rospy
import threading
import math
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class OdometryFilter(object):
    def __init__(self):
        self._lock = threading.Lock()

        try:
            _topic = rospy.get_param("~odometry_topic")
        except KeyError:
            rospy.logerr("OdometryFilter: unable to lookup parameter")
            raise

        self._odom_pub = rospy.Publisher(
            "pose/filtered", PoseWithCovarianceStamped, queue_size=10
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._sub = rospy.Subscriber(_topic, Odometry, self._odom_callback)

    def _odom_callback(self, msg):
        with self._lock:
            _time = msg.header.stamp

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    "front_track_camera_pose_frame",
                    _time,
                    rospy.Duration(1.0),
                )
            except Exception as e:
                rospy.logerr(
                    "Odom filter: error looking up transform {}".format(e)
                )
                raise

            _pose = PoseStamped()
            _pose.pose = msg.pose.pose

            final_pose = tf2_geometry_msgs.do_transform_pose(_pose, transform)

            final_msg = PoseWithCovarianceStamped()
            final_msg.header.frame_id = "odom"
            final_msg.header.stamp = _time
            final_msg.pose.pose = final_pose.pose
            final_msg.pose.covariance = msg.pose.covariance
            self._odom_pub.publish(final_msg)


if __name__ == "__main__":
    rospy.init_node("odom_filter")
    node_ = OdometryFilter()
    rospy.spin()
