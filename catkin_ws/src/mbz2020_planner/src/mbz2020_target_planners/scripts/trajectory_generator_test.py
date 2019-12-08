#!/usr/bin/env python

import rospy
import actionlib

from geometry_msgs.msg import Point

from mbz2020_planner.msg import PlanRequestAction, PlanRequestGoal
from mbz2020_common.msg import MotionTrajectory

def run(server_name):
    client = actionlib.SimpleActionClient(server_name, PlanRequestAction)

    _plan_pub = rospy.Publisher('motion_trajectory', MotionTrajectory, queue_size=10)

    rospy.loginfo('PlannerClient: Waiting for server')
    client.wait_for_server()
    rospy.loginfo('PlannerClient: server ready')

    _rate = rospy.Rate(10000)

    request = PlanRequestGoal()

    start = Point()

    middle = Point()
    middle.x = 3
    middle.y = 0
    middle.z = 1

    end = Point()
    end.x = 5
    end.y = 1
    end.z = 1

    request.waypoints.append(start)
    request.waypoints.append(middle)
    request.waypoints.append(end)

    count = 0
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        client.send_goal_and_wait(request)
        _result = client.get_result()
        # rospy.logwarn('Success: {}'.format(client.get_result().success))

        # if _result.success:
        #    _plan_pub.publish(_result.plan)
        count = count + 1

        # _rate.sleep()

    time_ = (rospy.Time.now() - start_time).to_sec()

    rospy.logerr('{} cycles in {} ms'.format(count, time_))
    rospy.logerr('Frequency is {}'.format(count/time_))

if __name__ == '__main__':
    rospy.init_node('test_plan_client')
    rospy.loginfo('Test Plan Client starting up')
    name = 'nonlinear_trajectory_request'
    run(name)
