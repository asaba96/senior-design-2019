#!/usr/bin/env python

import rospy
import actionlib

from robot_motions_server_ros.msg import TaskRequestAction, TaskRequestGoal


def run(server_name):
    client = actionlib.SimpleActionClient(server_name, TaskRequestAction)

    rospy.loginfo("TakeoffLandMission: Waiting for server")
    client.wait_for_server()
    rospy.loginfo("TakeoffLandMission: server ready")

    rospy.sleep(5)


    request = TaskRequestGoal(action_type="pickup_target")
    client.send_goal_and_wait(request)

    rospy.logwarn("Success: {}".format(client.get_result().success))


if __name__ == "__main__":
    rospy.init_node("takeoff_land_mission")

    rospy.loginfo("TakeoffLandMission: starting up...")
    name = rospy.get_param("/task_manager/action_server_name")
    run(name)
