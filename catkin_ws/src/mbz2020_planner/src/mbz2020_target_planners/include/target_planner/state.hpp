#ifndef STATE_HPP
#define STATE_HPP

// system and ros headers
#include <ros/ros.h>
#include <iostream>
#include <limits>
#include <functional>

// message headers
#include <mbz2020_common/MotionPointStamped.h>

using mbz2020_common::MotionPointStamped;

namespace Mbz2020Planner {
// struct to define state at each node
struct GroundedState {
    double x;
    double y;
    double z;

    double xVel;
    double yVel;
    double zVel;

    double time;

    bool operator==(const GroundedState& b) const
    {
        double diff = (std::fabs(x - b.x) +
                       std::fabs(y - b.y) +
                       std::fabs(z - b.z) +
                       std::fabs(xVel - b.xVel) +
                       std::fabs(yVel - b.yVel) +
                       std::fabs(zVel - b.zVel) +
                       std::fabs(time - b.time));

        return (diff > 100000 * std::numeric_limits<double>::epsilon());
    }

    MotionPointStamped toMotionPoint() {
        MotionPointStamped msg;
        msg.header.frame_id = "world";
        msg.header.stamp = ros::Time::now() + ros::Duration(time);

        msg.motion_point.pose.position.x = x;
        msg.motion_point.pose.position.y = y;
        msg.motion_point.pose.position.z = z;

        msg.motion_point.twist.linear.x = xVel;
        msg.motion_point.twist.linear.y = yVel;
        msg.motion_point.twist.linear.z = zVel;

        return msg;
    }
};

struct State {
    /*
     * realPose = dP * numActions
     * dP = 1/2 * maxAccel * dt^2
     * numActions = 2*velNum + accelAction
     *
     * realVel = dV * numActions
     * dV = maxAccel * dt
     * numAccel = velNum + accelAction
     *
     * realTime = time * dt
     *
     */

    int numX;
    int numY;
    int numZ;

    int numXVel;
    int numYVel;
    int numZVel;

    int time;

    bool operator==(const State& b) const
    {
        return (numX == b.numX &&
                numY == b.numY &&
                numZ == b.numZ &&
                numXVel == b.numXVel &&
                numYVel == b.numYVel &&
                numZVel == b.numZVel &&
                time == b.time);
    }
};
}

namespace std {
template<> struct hash<Mbz2020Planner::State>
{
    std::size_t operator()(const Mbz2020Planner::State& s) const
    {
        return (std::hash<int>{} (s.numX)
                ^ (std::hash<int>{} (s.numY) << 1)
                ^ (std::hash<int>{} (s.numZ) << 2)
                ^ (std::hash<int>{} (s.numXVel) << 3)
                ^ (std::hash<int>{} (s.numYVel) << 4)
                ^ (std::hash<int>{} (s.numZVel) << 5)
                ^ (std::hash<int>{} (s.time) << 6));
    }
};
}

#endif // STATE_HPP
