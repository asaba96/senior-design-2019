#!/usr/bin/env python

import rospy
import math

from base_task import BaseTask
from task_config.task_states import TaskRunning, TaskDone, TaskFailed, TaskAborted
from task_config.task_commands import NopCommand, VelocityCommand
from task_utilities.pid_controller import PidSettings, PidController

from geometry_msgs.msg import TwistStamped


class PickupTargetTaskState(object):
    INIT = 0
    DESCENDING = 1
    GRABBING = 2
    ASCENDING = 3
    DROPPING = 4


class PickupTargetTask(BaseTask):
    def __init__(self, request):
        super(BaseTask, self).__init__()

        try:
            x_pid_settings = PidSettings(rospy.get_param('~pickup_target_pid_settings/x_terms'))
            y_pid_settings = PidSettings(rospy.get_param('~pickup_target_pid_settings/y_terms'))
            self._MAX_HORIZ_SPEED = rospy.get_param('~max_translation_speed')
            self._DESCENT_VELOCITY = rospy.get_param('~pickup_descent_velocity')
            self._ASCENT_VELOCITY = rospy.get_param("~pickup_ascent_velocity")
            self._DONE_HEIGHT = rospy.get_param("~pickup_done_height")
            self._LOWER_HEIGHT_THRESHOLD = rospy.get_param("~lower_height_threshold")
            self._LANDED_HEIGHT = rospy.get_param("~landed_height")
        except KeyError:
            rospy.logerr('PickupTargetTask: could not lookup param')
            raise

        self._x_pid = PidController(x_pid_settings)
        self._y_pid = PidController(y_pid_settings)

        self._state = PickupTargetTaskState.INIT

    def get_desired_command(self):
        if self._state == PickupTargetTaskState.INIT:
            if not self.topic_buffer.has_target_msg() or \
               not self.topic_buffer.has_odometry_message() or \
               not self.topic_buffer.has_range() or \
               not self.topic_buffer.has_arm_info():
                rospy.logerr("PickupTargetTask: no target or odom messages, task aborting")
                return TaskAborted(), NopCommand()
            else:
                self._state = PickupTargetTaskState.DESCENDING

        target = self.topic_buffer.get_target_msg()
        odom = self.topic_buffer.get_odometry_message()
        arm_info = self.topic_buffer.get_arm_info()
        height = self.topic_buffer.get_height()

        # teraranger maxes out at .5
        if height <= .52:
            height = (arm_info.range1 + arm_info.range2)/2

        # if target has been obtained
        target_status = arm_info.motor_status

        self._check_height_transition(height, target_status)

        if self._state == PickupTargetTaskState.DESCENDING:
            x_p_diff = target.pose.pose.position.x
            y_p_diff = target.pose.pose.position.y

            x_success, x_response = self._x_pid.update(x_p_diff, rospy.Time.now(), False)
            y_success, y_response = self._y_pid.update(y_p_diff, rospy.Time.now(), False)

            # PID controller does setpoint - current;
            # TODO: see if the signs need flipped here
            # also see if we want to add FF terms
            if x_success:
                x_vel_target = -x_response
            else:
                x_vel_target = 0.0
                rospy.logerr("PickupTargetTask: error in PID update for X term")

            if y_success:
                y_vel_target = y_response
            else:
                y_vel_target = 0.0
                rospy.logerr("PickupTargetTask: error in PID update for Y term")

            # Cap the horizontal velocity
            h_vel_target = math.sqrt(x_vel_target**2 + y_vel_target**2)
            if h_vel_target > self._MAX_HORIZ_SPEED:
                x_vel_target = x_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)
                y_vel_target = y_vel_target * (self._MAX_HORIZ_SPEED/h_vel_target)

            msg = TwistStamped()
            msg.twist.linear.x = x_vel_target
            msg.twist.linear.y = y_vel_target
            msg.twist.linear.z = self._DESCENT_VELOCITY

            return TaskRunning(), VelocityCommand(target_twist=msg)

        if self._state == PickupTargetTaskState.GRABBING:
            msg = TwistStamped()
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = self._DESCENT_VELOCITY

            self.topic_buffer.send_motor_command(1)

            return TaskRunning(), VelocityCommand(target_twist=msg)

        if self._state == PickupTargetTaskState.ASCENDING:
            if height > self._DONE_HEIGHT:
                self._state = PickupTargetTaskState.DROPPING
                self.topic_buffer.send_motor_command(2)
                return TaskRunning(), VelocityCommand(target_twist=TwistStamped())

            msg = TwistStamped()
            msg.twist.linear.z = self._ASCENT_VELOCITY

            self.topic_buffer.send_motor_command(0)

            return TaskRunning(), VelocityCommand(target_twist=msg)

        if self._state == PickupTargetTaskState.DROPPING:
            # stop motor and hover
            self.topic_buffer.send_motor_command(0)
            return TaskDone(), VelocityCommand(target_twist=TwistStamped())

        return TaskRunning(), NopCommand()

    def cancel(self):
        return True

    # gets distance between two Points
    def _get_distance(self, A, B):
        return math.sqrt(A.x**2 + B.x**2)

    def _check_height_transition(self, height, status):
        if self._state == PickupTargetTaskState.DESCENDING:
            if height < self._LOWER_HEIGHT_THRESHOLD:
                self._state = PickupTargetTaskState.GRABBING
        elif self._state == PickupTargetTaskState.GRABBING:
            # check for status for switching to ascend
            # if below landed height or have target, go back up
            if height < self._LANDED_HEIGHT or status == 1:
                self._state = PickupTargetTaskState.ASCENDING
