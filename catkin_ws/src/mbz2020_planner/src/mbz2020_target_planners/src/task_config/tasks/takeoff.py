#!/usr/bin/env python

import rospy

from base_task import BaseTask
from task_config.task_states import TaskRunning, TaskDone, TaskFailed, TaskAborted
from task_config.task_commands import NopCommand, PoseCommand


class TakeoffTask(BaseTask):
    def __init__(self, request):
        super(BaseTask, self).__init__()
        try:
            self._TAKEOFF_VEL = rospy.get_param("~takeoff_velocity")
            self._DT = rospy.get_param("~takeoff_dt")
            self._TAKEOFF_DONE_HEIGHT = rospy.get_param(
                "~takeoff_complete_height"
            )
        except KeyError:
            rospy.logerr("TakeoffTask: could not lookup param")
            raise

        self._DP = self._TAKEOFF_VEL * self._DT

    def get_desired_command(self):
        if not self.topic_buffer.has_odometry_message():
            rospy.logerr(
                "TakeoffTask: no odom messages available, task aborting"
            )
            return TaskAborted(), NopCommand()

        odom = self.topic_buffer.get_odometry_message()
        curr_z = odom.pose.pose.position.z
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y

        if curr_z >= self._TAKEOFF_DONE_HEIGHT:
            rospy.loginfo(
                "TakeoffTask: above takeoff done height, took off successfully"
            )
            return TaskDone(), NopCommand()

        return TaskRunning(), PoseCommand(curr_x, curr_y, curr_z + self._DP)

    def cancel(self):
        return True
