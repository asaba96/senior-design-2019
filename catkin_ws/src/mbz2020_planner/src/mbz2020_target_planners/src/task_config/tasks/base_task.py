import rospy

from task_utilities.topic_buffer import TopicBuffer
from motions_server.abstract_task import AbstractTask


class BaseTask(AbstractTask):
    topic_buffer = None

    def __init__(self):
        if BaseTask.topic_buffer is None:
            BaseTask.topic_buffer = TopicBuffer()
        self.topic_buffer = BaseTask.topic_buffer
