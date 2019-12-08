#!/usr/bin/env python

import rospy

# import all tasks
from tasks.base_task import BaseTask
from tasks.land import LandTask
from tasks.takeoff import TakeoffTask
from tasks.xyz_translate import XYZTranslateTask
from tasks.go_to_target import GoToTargetTask
from tasks.target_search import TargetSearchTask
from tasks.pickup_target import PickupTargetTask

# task states and commands, which have their own appropriate handlers
from task_states import TaskRunning, TaskDone, TaskAborted, TaskFailed
from task_commands import BaseCommand

from motions_server.abstract_command_handler import (
    TaskHandledState,
    AbstractCommandHandler,
)
from motions_server.abstract_safety_responder import AbstractSafetyResponder


class CommandHandler(AbstractCommandHandler):
    def __init__(self):
        self._task_dict = {
            "takeoff": TakeoffTask,
            "land": LandTask,
            "xyztranslate": XYZTranslateTask,
            "go_to_target": GoToTargetTask,
            "target_search": TargetSearchTask,
            "pickup_target": PickupTargetTask,
        }

    def check_request(self, request):
        try:
            new_task_type = self._task_dict[request.action_type]
        except KeyError as e:
            rospy.logerr(
                "TaskConfig: invalid action type: %s", request.action_type
            )
            return None

        try:
            new_task = new_task_type(request)
        except Exception as e:
            rospy.logerr(
                "TaskConfig: Could not construct task: %s", request.action_type
            )
            rospy.logerr(str(e))
            return None

        return new_task

    def wait_until_ready(self, timeout):
        BaseTask().topic_buffer.wait_until_ready(timeout)
        BaseCommand().command_buffer.wait_until_ready(timeout)

    def handle(self, command, state):
        try:
            success = command.handle()
        except Exception as e:
            rospy.logerr("TaskConfig: Could not handle task command")
            rospy.logerr(str(e))
            return TaskHandledState.ABORTED

        if not success:
            rospy.logerr("TaskConfig: Command handle returned failure, failing task")
            return TaskHandledState.FAILED

        if isinstance(state, TaskRunning):
            return TaskHandledState.OKAY
        elif isinstance(state, TaskDone):
            return TaskHandledState.DONE
        elif isinstance(state, TaskAborted):
            return TaskHandledState.ABORTED
        elif isinstance(state, TaskFailed):
            return TaskHandledState.FAILED
        else:
            rospy.logerr("CommandHandler: Task provided invalid state")
            return TaskHandledState.ABORTED


class SafetyResponder(AbstractSafetyResponder):
    def __init__(self):
        pass

    def activate_safety_response(self):
        return

    def wait_until_ready(self, timeout):
        return
