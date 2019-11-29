#!/usr/bin/env python

import rospy

from std_msgs.msg import Int8
from mbz2020_common.msg import MultiRange

status = None

def callback(msg):
    global status
    status = msg.motor_status

if __name__ == '__main__':
    rospy.init_node('demo')

    pub = rospy.Publisher('motor_cmd', Int8, queue_size=1000)
    sub = rospy.Subscriber("multi_range", MultiRange, callback)

    msg = Int8()
    msg.data = 2

    pub.publish(msg)

    rospy.logwarn('Starting Motor....')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown() and status is None:
        pub.publish(msg)
        rospy.logwarn_throttle(1, "Waiting on status")
        rate.sleep()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and status == 0:
        rospy.logwarn_throttle(1, "Waiting on ball...")
        pub.publish(msg)
        rate.sleep()

    rospy.logwarn("Ball retrieved...")

    msg.data = 0
    pub.publish(msg)

    rospy.sleep(2)

    rospy.logwarn("Ejecting ball...")

    msg.data = 1
    pub.publish(msg)
    pub.publish(msg)

    rospy.sleep(4)

    rospy.logwarn("Stopping motors...")

    msg.data = 0
    pub.publish(msg)
    pub.publish(msg)

    rospy.logwarn("Done")
