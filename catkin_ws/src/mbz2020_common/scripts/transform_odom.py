#!/usr/bin/env python

import rospy
import threading
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped

class TransformOdomNode(object):
    def __init__(self):
        self._sub = rospy.Subscriber(
                'front_track_camera/odom/sample',
                Odometry,
                self._odom_callback)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._transform_broadcaster = tf2_ros.TransformBroadcaster()

        self._lock = threading.Lock()

    def _odom_callback(self, msg):
        with self._lock:
            try:
                transform = self.tf_buffer.lookup_transform(
                                            'base_link',
                                            msg.child_frame_id,
                                            rospy.Time.now(),
                                            rospy.Duration(1.0))
            except Exception as e:
                rospy.logerr('ERROR in TransformOdomNode: ' + str(e))
                raise

            stamped_point = PoseStamped()
            stamped_point.pose = msg.pose.pose

            final_pose = tf2_geometry_msgs.do_transform_pose(stamped_point, transform)

            msg_final = self._construct_transform(final_pose, msg)
            self._transform_broadcaster.sendTransform(msg_final)

    def _construct_transform(self, odom, msg):
        transform_msg = TransformStamped()

        transform_msg.header.stamp = msg.header.stamp
        transform_msg.header.frame_id = 'odom'
        transform_msg.child_frame_id = 'base_link'

        transform_msg.transform.rotation = odom.pose.orientation

        transform_msg.transform.translation.x = odom.pose.position.x
        transform_msg.transform.translation.y = odom.pose.position.y
        transform_msg.transform.translation.z = odom.pose.position.z

        print(transform_msg)

        return transform_msg

if __name__ == '__main__':
    rospy.init_node('transform_odom')

    node_ = TransformOdomNode()

    rospy.spin()
