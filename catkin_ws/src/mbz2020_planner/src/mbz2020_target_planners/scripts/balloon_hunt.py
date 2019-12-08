#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, String
from tf.transformations import quaternion_from_euler as e2q
from math import pi
from mbz2020_common.msg import TargetArray

import numpy as np


class BalloonHunt(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.quick_user_interrupt = False

        # variables
        self.cmd_state = ""
        self.hunt_status = -1  # -1: Idle, 0: Working, 1: Done
        self.attack_status = -1
        self.odom = Odometry()
        # self.balloon_detected = -1 # -1: Unknown, 0: No detection, 1: Balloon detected
        self.balloon = TargetArray()

        self.tolerance = [
            0.5,
            0.5,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
        ]  # pose error tolerance in p_x, p_y, p_z, q_x, q_y, q_z, q_w

        # publishers
        self.pub_traj = rospy.Publisher(
            "/firefly/command/trajectory",
            MultiDOFJointTrajectory,
            queue_size=10,
        )
        self.pub_hunt_status = rospy.Publisher(
            "/mission/planner_hunt/status", Int16, queue_size=10
        )
        self.pub_attack_status = rospy.Publisher(
            "/mission/planner_attack/status", Int16, queue_size=10
        )

        # subscribers
        self.sub_cmd_state = rospy.Subscriber(
            "/mission/command/state", String, self.cmd_state_callback
        )
        self.sub_odom = rospy.Subscriber(
            "/firefly/odometry_sensor1/odometry", Odometry, self.odom_callback
        )
        self.sub_balloon = rospy.Subscriber(
            "/perception/balloon", TargetArray, self.balloon_callback
        )

    def run(self):
        rate = rospy.Rate(30)

        step_size = 0.2
        width = 10.0
        depth = 2.0
        x, y, z, q_x, q_y, q_z, q_w = self.generate_route(
            step_size, width, depth
        )

        # track progress along hunting route
        n_steps = x.size
        step = 0

        while not rospy.is_shutdown():
            # rospy.loginfo((x[step], y[step], z[step]))

            msg_status = Int16()
            msg_status.data = self.hunt_status
            self.pub_hunt_status.publish(msg_status)

            msg_status_attack = Int16()
            msg_status_attack.data = self.attack_status
            self.pub_attack_status.publish(msg_status_attack)

            if self.cmd_state == "Hunt":

                self.hunt_status = 0
                self.attack_status = -1

                if step == n_steps - 1:
                    if self.has_reached(
                        (x[-1], y[-1], z[-1]),
                        (q_x[-1], q_y[-1], q_z[-1], q_w[-1]),
                    ):

                        self.hunt_status = 1

                    else:

                        self.command_pose(
                            (x[step], y[step], z[step]),
                            (q_x[step], q_y[step], q_z[step], q_w[step]),
                        )

                else:
                    if not self.has_reached(
                        (x[step], y[step], z[step]),
                        (q_x[step], q_y[step], q_z[step], q_w[step]),
                    ):
                        self.command_pose(
                            (x[step], y[step], z[step]),
                            (q_x[step], q_y[step], q_z[step], q_w[step]),
                        )
                    else:
                        step = step + 1

            elif self.cmd_state == "Attack":

                self.hunt_status = 0

                balloon_x = self.balloon.targets[0].odom.pose.pose.position.x
                balloon_y = self.balloon.targets[0].odom.pose.pose.position.y
                balloon_z = self.balloon.targets[0].odom.pose.pose.position.z

                if self.has_reached(
                    (balloon_x, balloon_y, balloon_z),
                    (q_x[step], q_y[step], q_z[step], q_w[step]),
                ):

                    self.attack_status = 1
                    rospy.loginfo("Balloon popped!")

                else:

                    self.attack_status = 0
                    drone_x = self.odom.pose.pose.position.x
                    drone_y = self.odom.pose.pose.position.y
                    drone_z = self.odom.pose.pose.position.z

                    balloon_step = 0.4

                    command_x = (
                        balloon_x
                        if (np.abs(drone_x - balloon_x) < balloon_step)
                        else drone_x
                        + np.sign(balloon_x - drone_x) * balloon_step
                    )
                    command_y = (
                        balloon_y
                        if (np.abs(drone_y - balloon_y) < balloon_step)
                        else drone_y
                        + np.sign(balloon_y - drone_y) * balloon_step
                    )
                    command_z = (
                        balloon_z
                        if (np.abs(drone_z - balloon_z) < balloon_step)
                        else drone_z
                        + np.sign(balloon_z - drone_z) * balloon_step
                    )

                    self.command_pose(
                        (command_x, command_y, command_z),
                        (q_x[step], q_y[step], q_z[step], q_w[step]),
                    )

            else:

                self.hunt_status = -1
                self.attack_status = -1

            rate.sleep()

    def has_reached(self, position, orientation):
        px_done = (
            abs(self.odom.pose.pose.position.x - position[0])
            < self.tolerance[0]
        )
        py_done = (
            abs(self.odom.pose.pose.position.y - position[1])
            < self.tolerance[1]
        )
        pz_done = (
            abs(self.odom.pose.pose.position.z - position[2])
            < self.tolerance[2]
        )
        qx_done = (
            abs(self.odom.pose.pose.orientation.x - orientation[0])
            < self.tolerance[3]
        )
        qy_done = (
            abs(self.odom.pose.pose.orientation.y - orientation[1])
            < self.tolerance[4]
        )
        qz_done = (
            abs(self.odom.pose.pose.orientation.z - orientation[2])
            < self.tolerance[5]
        )
        qw_done = (
            abs(self.odom.pose.pose.orientation.w - orientation[3])
            < self.tolerance[6]
        )
        return (
            px_done
            and py_done
            and pz_done
            and qx_done
            and qy_done
            and qz_done
            and qw_done
        )

    def command_pose(self, position, orientation):
        msg_traj = MultiDOFJointTrajectory()
        msg_traj.header.stamp = rospy.Time()
        msg_traj.joint_names = ["base_link"]

        p = Vector3(position[0], position[1], position[2])
        q = Quaternion(
            orientation[0], orientation[1], orientation[2], orientation[3]
        )
        transforms = [Transform(translation=p, rotation=q)]
        point = MultiDOFJointTrajectoryPoint(
            transforms, [Twist()], [Twist()], rospy.Time()
        )
        msg_traj.points.append(point)

        self.pub_traj.publish(msg_traj)

    def generate_route(self, step_size, width, depth):
        x_right = np.arange(0.0, width + step_size, step_size)
        y_forward = np.arange(0.0, depth + step_size, step_size)
        x_forward = np.ones_like(y_forward) * width
        x_left = x_right[::-1]
        x_back = np.zeros_like(y_forward)
        y_right = np.zeros_like(x_right)
        y_left = np.ones_like(x_right) * depth
        y_back = y_forward[::-1]

        yaw_delay = 10
        z_setpt = 2.0

        x_right1 = np.concatenate((np.ones(yaw_delay) * 0.0, x_right))
        x_forward1 = np.concatenate(
            (np.ones(yaw_delay) * x_right[-1], x_forward)
        )
        x_left1 = np.concatenate((np.ones(yaw_delay) * x_forward[-1], x_left))
        x_forward12 = np.concatenate((np.ones(yaw_delay) * x_left[-1], x_back))
        x_right2 = np.concatenate((np.ones(yaw_delay) * 0.0, x_right))
        x_forward2 = np.concatenate(
            (np.ones(yaw_delay) * x_right[-1], x_forward)
        )
        x_left2 = np.concatenate((np.ones(yaw_delay) * x_forward[-1], x_left))
        x_forward22 = np.concatenate((np.ones(yaw_delay) * x_left[-1], x_back))

        y_right1 = np.concatenate((np.ones(yaw_delay) * 0.0, y_right))
        y_forward1 = np.concatenate(
            (np.ones(yaw_delay) * y_right[-1], y_forward)
        )
        y_left1 = np.concatenate((np.ones(yaw_delay) * y_forward[-1], y_left))
        y_forward12 = np.concatenate(
            (np.ones(yaw_delay) * y_left[-1], y_back[::-1] + 1 * depth)
        )
        y_right2 = np.concatenate(
            (np.ones(yaw_delay) * y_back[0] + 1 * depth, y_right + 2 * depth)
        )
        y_forward2 = np.concatenate(
            (
                np.ones(yaw_delay) * y_right[-1] + 2 * depth,
                y_forward + 2 * depth,
            )
        )
        y_left2 = np.concatenate(
            (np.ones(yaw_delay) * y_forward[-1] + 2 * depth, y_left + 2 * depth)
        )
        y_forward22 = np.concatenate(
            (
                np.ones(yaw_delay) * y_left[-1] + 2 * depth,
                y_back[::-1] + 3 * depth,
            )
        )

        x = np.concatenate(
            (
                x_right1,
                x_forward1,
                x_left1,
                x_forward12,
                x_right2,
                x_forward2,
                x_left2,
                x_forward22,
            )
        )

        y = np.concatenate(
            (
                y_right1,
                y_forward1,
                y_left1,
                y_forward12,
                y_right2,
                y_forward2,
                y_left2,
                y_forward22,
            )
        )

        z = np.ones_like(x) * z_setpt

        q_east = e2q(0, 0, 0)
        q_north = e2q(0, 0, pi / 2)
        q_west = e2q(0, 0, pi)

        q_x = np.concatenate(
            (
                np.ones_like(x_right1) * q_east[0],
                np.ones_like(x_forward1) * q_north[0],
                np.ones_like(x_left1) * q_west[0],
                np.ones_like(x_forward12) * q_north[0],
                np.ones_like(x_right2) * q_east[0],
                np.ones_like(x_forward2) * q_north[0],
                np.ones_like(x_left2) * q_west[0],
                np.ones_like(x_forward22) * q_north[0],
            )
        )

        q_y = np.concatenate(
            (
                np.ones_like(x_right1) * q_east[1],
                np.ones_like(x_forward1) * q_north[1],
                np.ones_like(x_left1) * q_west[1],
                np.ones_like(x_forward12) * q_north[1],
                np.ones_like(x_right2) * q_east[1],
                np.ones_like(x_forward2) * q_north[1],
                np.ones_like(x_left2) * q_west[1],
                np.ones_like(x_forward22) * q_north[1],
            )
        )

        q_z = np.concatenate(
            (
                np.ones_like(x_right1) * q_east[2],
                np.ones_like(x_forward1) * q_north[2],
                np.ones_like(x_left1) * q_west[2],
                np.ones_like(x_forward12) * q_north[2],
                np.ones_like(x_right2) * q_east[2],
                np.ones_like(x_forward2) * q_north[2],
                np.ones_like(x_left2) * q_west[2],
                np.ones_like(x_forward22) * q_north[2],
            )
        )

        q_w = np.concatenate(
            (
                np.ones_like(x_right1) * q_east[3],
                np.ones_like(x_forward1) * q_north[3],
                np.ones_like(x_left1) * q_west[3],
                np.ones_like(x_forward12) * q_north[3],
                np.ones_like(x_right2) * q_east[3],
                np.ones_like(x_forward2) * q_north[3],
                np.ones_like(x_left2) * q_west[3],
                np.ones_like(x_forward22) * q_north[3],
            )
        )

        return x, y, z, q_x, q_y, q_z, q_w

    def cmd_state_callback(self, msg):
        self.cmd_state = msg.data

    def odom_callback(self, msg):
        self.odom = msg

    def balloon_callback(self, msg):
        self.balloon = msg

    def shutdown(self):
        # unregister subscribers
        self.sub_cmd_state.unregister()
        self.sub_odom.unregister()
        self.sub_balloon.unregister()

        # kill
        self.quick_user_interrupt = True


if __name__ == "__main__":
    try:
        rospy.init_node("ballon_hunt", anonymous=True)
        BALLOON_HUNT = BalloonHunt()
        BALLOON_HUNT.run()
    except rospy.ROSInterruptException:
        pass
