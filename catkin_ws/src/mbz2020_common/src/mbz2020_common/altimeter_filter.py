#!/usr/bin/env python

import rospy
import threading
import math
import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped


class AltimeterFilter(object):
    def __init__(self):
        self._lock = threading.Lock()

        try:
            _topic = rospy.get_param("~altimeter_topic")
            self._range_min = rospy.get_param("~altimeter_min")
            self._range_max = rospy.get_param("~altimeter_max")
        except KeyError:
            rospy.logerr("AltimeterFilter: unable to lookup parameter")
            raise

        self._range_pub = rospy.Publisher(
            "altimeter", PoseWithCovarianceStamped, queue_size=10
        )

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._sub = rospy.Subscriber(
            _topic, PoseWithCovarianceStamped, self._range_callback
        )

    def _range_callback(self, msg):
        with self._lock:
            z = msg.pose.pose.position.z
            _time = msg.header.stamp
            if math.isinf(z) or z < self._range_min or z > self._range_max:
                rospy.logerr("AltimeterFilter: rejecting range of {}".format(z))
                return

            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link", msg.header.frame_id, _time, rospy.Duration(1.0)
                )
            except Exception as e:
                rospy.logerr(
                    "AltimeterFilter: error looking up transform {}".format(e)
                )
                raise

            stamped_point = PointStamped()
            stamped_point.point.z = z

            final_pose = tf2_geometry_msgs.do_transform_point(
                stamped_point, transform
            )

            final_msg = PoseWithCovarianceStamped()
            final_msg.header.frame_id = "odom"
            final_msg.header.stamp = _time
            final_msg.pose.pose.position.z = final_pose.point.z
            final_msg.pose.covariance = msg.pose.covariance
            self._range_pub.publish(final_msg)


if __name__ == "__main__":
    rospy.init_node("altimeter_filter")
    node_ = AltimeterFilter()
    rospy.spin()
