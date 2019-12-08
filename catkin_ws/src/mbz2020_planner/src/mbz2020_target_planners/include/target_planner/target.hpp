#ifndef TARGET_HPP
#define TARGET_HPP

// system and ros headers
#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

// ros msg headers
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <mbz2020_common/MotionTrajectory.h>

using nav_msgs::Odometry;
using geometry_msgs::Point;
using geometry_msgs::Vector3;
using mbz2020_common::MotionTrajectory;

namespace Mbz2020Planner {

class Target {
public:
    Target(MotionTrajectory trajectory);
    Target(ros::NodeHandle& nh);
    Odometry getTargetAtTime(double time);
    bool hasTargetMessage();
private:
    ros::Subscriber targetSub;

    void targetCallback(const nav_msgs::Odometry::ConstPtr& msg);
    nav_msgs::Odometry currOdom;
    MotionTrajectory _trajectory;

    bool _isStatic;
    bool _hasTargetMsg;

};
}
#endif // TARGET_HPP
