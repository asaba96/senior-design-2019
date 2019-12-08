#!/usr/bin/env python

import rospy
import actionlib

from robot_motions_server_ros.msg import TaskRequestAction, TaskRequestGoal


def run(server_name):
    client = actionlib.SimpleActionClient(server_name, TaskRequestAction)

    rospy.loginfo("TaskServer: Waiting for server")
    client.wait_for_server()
    rospy.loginfo("TaskServer: server ready")

    request = TaskRequestGoal(action_type="takeoff")
    client.send_goal_and_wait(request)

    rospy.logwarn("Success: {}".format(client.get_result().success))

    request = TaskRequestGoal(action_type="land")
    client.send_goal_and_wait(request)

    rospy.logwarn("Success: {}".format(client.get_result().success))


if __name__ == "__main__":
    rospy.init_node("test_server")

    rospy.loginfo("TestServer: starting up...")
    name = rospy.get_param("/task_manager/action_server_name")
    run(name)
