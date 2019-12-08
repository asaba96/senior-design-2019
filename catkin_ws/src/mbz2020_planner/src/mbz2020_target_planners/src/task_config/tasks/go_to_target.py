#!/usr/bin/env python

import rospy

import actionlib

from geometry_msgs.msg import Point

from mbz2020_target_planners.msg import PlanRequestAction, PlanRequestGoal
from mbz2020_common.msg import MotionTrajectory

from base_task import BaseTask
from task_config.task_states import TaskRunning, TaskDone, TaskFailed, TaskAborted
from task_config.task_commands import NopCommand, PoseCommand, TrajectoryCommand


class GoToTargetTask(BaseTask):
    def __init__(self, request):
        super(BaseTask, self).__init__()
        self._plan_sent = False
        self._plan = None

    def _done_cb(self, status, msg):
        self._plan = msg.plan

    def get_desired_command(self):
        if not self.topic_buffer.has_target_msg() or not self.topic_buffer.has_odometry_message():
            rospy.logerr("GoToTargetTask: no target or odom messages available, task aborting")
            return TaskAborted(), NopCommand()

        if self._plan_sent:
            if self._plan is not None:
                return TaskRunning(), TrajectoryCommand(self._plan)
        else:
            odom = self.topic_buffer.get_odometry_message()
            curr_z = odom.pose.pose.position.z
            curr_x = odom.pose.pose.position.x
            curr_y = odom.pose.pose.position.y

            target = self.topic_buffer.get_target_msg()
            request = PlanRequestGoal()

            start = Point()
            start.x = curr_x
            start.y = curr_y
            start.z = curr_z

            end = Point()
            end.x = target.pose.pose.position.x
            end.y = target.pose.pose.position.y
            end.z = curr_z

            request.waypoints.append(start)
            request.waypoints.append(end)

            self.topic_buffer.make_traj_plan_request(request, self._done_cb)
            self._plan_sent = True

        return TaskRunning(), NopCommand()

    def cancel(self):
        return True
