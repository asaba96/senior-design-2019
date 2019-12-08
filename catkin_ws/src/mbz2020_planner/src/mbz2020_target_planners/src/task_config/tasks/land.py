#!/usr/bin/env python

import rospy

from base_task import BaseTask
from task_config.task_states import TaskRunning, TaskDone, TaskFailed, TaskAborted
from task_config.task_commands import NopCommand, PoseCommand


class LandTask(BaseTask):
    def __init__(self, request):
        super(BaseTask, self).__init__()
        try:
            self._LANDING_VEL = rospy.get_param("~landing_velocity")
            self._DT = rospy.get_param("~landing_dt")
            self._LANDING_DONE_HEIGHT = rospy.get_param(
                "~landing_complete_height"
            )
        except KeyError:
            rospy.logerr("LandTask: could not lookup param")
            raise

        self._DP = self._LANDING_VEL * self._DT

    def get_desired_command(self):
        if not self.topic_buffer.has_odometry_message():
            rospy.logerr("LandTask: no odom messages available, task aborting")
            return TaskAborted(), NopCommand()

        # TODO: use base_footprint transform instead of odom
        odom = self.topic_buffer.get_odometry_message()
        curr_z = odom.pose.pose.position.z
        curr_x = odom.pose.pose.position.x
        curr_y = odom.pose.pose.position.y

        if curr_z < self._LANDING_DONE_HEIGHT:
            rospy.loginfo(
                "LandTask: below landing done height, landing successful"
            )
            return TaskDone(), NopCommand()

        return TaskRunning(), PoseCommand(curr_x, curr_y, curr_z - self._DP)

    def cancel(self):
        return True
