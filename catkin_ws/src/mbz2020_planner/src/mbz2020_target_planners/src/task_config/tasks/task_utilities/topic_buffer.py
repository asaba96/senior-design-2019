import rospy
import tf2_ros
import math
import actionlib

from mbz2020_common.msg import MultiRange
from mbz2020_target_planners.msg import PlanRequestAction

from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class TopicBuffer(object):
    def __init__(self):
        self._drone_odometry = None
        self._target_msg = None
        self._tag_msg = None
        self._range = None
        self._arm_info = None

        try:
            server_name = rospy.get_param("~traj_server")
            odom_topic = rospy.get_param("~odom_topic")
            target_topic = rospy.get_param("~target_topic")
            tag_topic = rospy.get_param("~tag_topic")
            self._USE_TAG = rospy.get_param("~use_tag")
        except KeyError:
            rospy.logerr('Topic Buffer: cannot find parameter')
            raise

        self._current_velocity_sub = rospy.Subscriber(
            odom_topic,
            Odometry,
            self._current_velocity_callback,
        )

        self._target_sub = rospy.Subscriber(
            target_topic,
            Odometry,
            self._target_callback,
        )

        self._tag_sub = rospy.Subscriber(
            tag_topic,
            PoseStamped,
            self._tag_callback,
        )

        self._range_sub = rospy.Subscriber(
            '/teraranger_evo',
            Range,
            self._range_callback,
        )

        self._arm_sub = rospy.Subscriber(
            '/multi_range',
            MultiRange,
            self._arm_callback
        )

        self._motor_pub = rospy.Publisher(
            '/motor_cmd',
            Int8,
            queue_size=100
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._task_message_dictionary = {}

        self._traj_gen_client = actionlib.SimpleActionClient(server_name, PlanRequestAction)

        # if not self._traj_gen_client.wait_for_server(rospy.Duration(20)):
        #    rospy.logerr('TopicBuffer: timed out waiting for server')
        #    raise RuntimeError('Timed out')

    def wait_until_ready(self, timeout):
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)
        while (
            not rospy.is_shutdown() and rospy.Time.now() - start_time < timeout
        ):
            if self.has_odometry_message() and self.has_range():
                return
            rate.sleep()
        rospy.logerr("TopicBuffer: Have odom: %s", self.has_odometry_message())
        rospy.logerr("TopicBuffer: Have range: %s", self.has_range())

        raise RuntimeError("TopicBuffer: not all dependencies met in time")

    def _current_velocity_callback(self, data):
        self._drone_odometry = data

    def has_odometry_message(self):
        return self._drone_odometry is not None

    def get_odometry_message(self):
        return self._drone_odometry

    def get_tf_buffer(self):
        return self._tf_buffer

    def get_task_message_dictionary(self):
        return self._task_message_dictionary

    def make_traj_plan_request(self, request, fb_callback):
        self._traj_gen_client.send_goal(request, done_cb=fb_callback)

    def _target_callback(self, msg):
        self._target_msg = msg

    def _tag_callback(self, msg):
        self._tag_msg = msg

    def has_target_msg(self):
        if self._USE_TAG:
            return self._tag_msg is not None
        else:
            return self._target_msg is not None

    def get_target_msg(self):
        if self._USE_TAG:
            msg = Odometry()
            msg.pose.pose = self._tag_msg.pose
            return msg
        else:
            return self._target_msg

    def _range_callback(self, msg):
        if not math.isnan(msg.range) and not math.isinf(msg.range):
            self._range = msg

    def get_height(self):
        return self._range.range

    def has_range(self):
        return self._range is not None

    def _arm_callback(self, msg):
        self._arm_info = msg

    def get_arm_info(self):
        return self._arm_info

    def has_arm_info(self):
        return self._arm_info is not None

    # 0 to stop, 1 to drop, 2 to pickup
    def send_motor_command(self, cmd):
        msg = Int8()
        msg.data = cmd
        self._motor_pub.publish(msg)
