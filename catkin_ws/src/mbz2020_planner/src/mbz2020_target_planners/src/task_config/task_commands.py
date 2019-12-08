#!/usr/bin/env python
import rospy
import traceback

from geometry_msgs.msg import TwistStamped, Vector3
from mbz2020_common.msg import MotionPointStamped, MotionTrajectory

from command_buffer import CommandBuffer
from linear_motion_profile_generator import LinearMotionProfileGenerator as LMPG


class BaseCommand(object):
    command_buffer = None

    def __init__(self):
        if BaseCommand.command_buffer is None:
            BaseCommand.command_buffer = CommandBuffer()

        self.command_buffer = BaseCommand.command_buffer
        self.motion_gen = LMPG.get_linear_motion_profile_generator()

    def handle(self):
        raise NotImplementedError("BaseCommand: method not implemented")


class TrajectoryCommand(BaseCommand):
    def __init__(self, trajectory):
        super(TrajectoryCommand, self).__init__()
        self.trajectory = trajectory

    def handle(self):
        try:
            self.command_buffer.publish_trajectory(self.trajectory)
        except Exception as e:
            rospy.logerr("TrajectoryCommand: error in handling command")
            rospy.logerr(str(e))
            return False

        return True


class PoseCommand(BaseCommand):
    def __init__(self, x, y, z, time_=None):
        super(PoseCommand, self).__init__()

        self.motion_point = MotionPointStamped()
        self.motion_point.motion_point.pose.position.x = x
        self.motion_point.motion_point.pose.position.y = y
        self.motion_point.motion_point.pose.position.z = z

        if time_ is None:
            self.motion_point.header.stamp = rospy.Time.now()
        else:
            self.motion_point.header.stamp = time_

    def handle(self):
        try:
            # TODO is how we want to do this?
            # Or do we want to publish most recent point?
            # Or should we send the whole trajectory?
            self.motion_gen.set_motion_point(self.motion_point)
            self.command_buffer.publish_point(self.motion_point)
        except Exception as e:
            rospy.logerr("PoseCommand: error in handling command")
            rospy.logerr(str(e))
            rospy.logerr(traceback.format_exc())
            return False

        return True


# TODO: fix to allow direct velocity command
class VelocityCommand(BaseCommand):
    def __init__(self,
                 target_twist=None,
                 start_position_x=None,
                 start_position_y=None,
                 start_position_z=None,
                 start_velocity_x=None,
                 start_velocity_y=None,
                 start_velocity_z=None,
                 acceleration=None):
        super(VelocityCommand, self).__init__()

        self.start_position = Vector3()
        self.start_position.x = start_position_x
        self.start_position.y = start_position_y
        self.start_position.z = start_position_z

        self.start_velocity = Vector3()
        self.start_velocity.x = start_velocity_x
        self.start_velocity.y = start_velocity_y
        self.start_velocity.z = start_velocity_z

        self.acceleration = acceleration

        if target_twist is None:
            self.target_twist = TwistStamped()
            self.target_twist.header.stamp = rospy.Time.now()
        else:
            self.target_twist = target_twist

    def handle(self):
        # current platform supports velocity directly
        # maybe support dynamic switching between two handle functions?
        self.command_buffer.publish_velocity(self.target_twist)
        return True

    def handle_old(self):
        try:
            plan, pose_only = self.motion_gen.get_velocity_plan(self)
        except Exception as e:
            rospy.logerr("VelocityCommand: error in handling command")
            rospy.logerr(str(e))
            return False

        if self._motion_gen.is_relative():
            rospy.logerr_throttle(1, 'VelocityCommand: motion profile generator does not have an absolute pose')

        traj = MotionTrajectory()
        traj.motion_points = plan

        self.command_buffer.publish_trajectory(traj)

        return True


class NopCommand(BaseCommand):
    def __init__(self):
        pass

    def handle(self):
        return True
