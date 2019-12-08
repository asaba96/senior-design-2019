// ROS headers
#include <ros/ros.h>

// message headers

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <mbz2020_common/MotionPoint.h>
#include <mbz2020_common/MotionPointStamped.h>
#include <mbz2020_common/MotionPointStampedArray.h>
#include <mbz2020_common/MotionTrajectory.h>

// planner headers
#include <target_planner/target_planner.hpp>

using geometry_msgs::PoseStamped;
using mbz2020_common::MotionPoint;
using mbz2020_common::MotionPointStamped;
using mbz2020_common::MotionPointStampedArray;
using mbz2020_common::MotionTrajectory;
using nav_msgs::Path;

using namespace Mbz2020Planner;

nav_msgs::OdometryConstPtr curr_odom;

void odomCallback(const nav_msgs::OdometryConstPtr msg) {
    curr_odom = msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "planning_server");

    ROS_INFO("Planner Demo Node starting up...");

    ros::NodeHandle nh;

    // private node handle has a specific namespace that allows us to easily
    // encapsulate parameters
    ros::NodeHandle private_nh("~");

    // PARAMETERS
    // TODO : make planner and node params pull from rosparam
    double update_frequency = 1;

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    ros::Publisher plan_pub = nh.advertise<nav_msgs::Path>("path_planner/path", 1000);
    ros::Publisher point_pub =
        nh.advertise<MotionTrajectory>("path_planner/plan", 1000);

    ros::Subscriber sub = nh.subscribe("/firefly/odometry_sensor1/odometry", 1000, odomCallback);

    // update frequency of the node
    ros::Rate rate(update_frequency);

    std::shared_ptr<Target> target_ = std::make_shared<Target>(nh);
    std::shared_ptr<Map> map_ = std::make_shared<Map>(private_nh, .1);
    std::shared_ptr<Limits> limits_ = std::make_shared<Limits>();

    limits_->maxX = 20.0;
    limits_->maxY = 20.0;
    limits_->maxZ = 20.0;

    limits_->minX = -20.0;
    limits_->minY = -20.0;
    limits_->minZ = 0.0;

    limits_->maxVel = 10.0;
    limits_->maxAccel = 5.0;

    limits_->maxTime = 200.0;

    double eps = 3.5;

    private_nh.getParam("/eps", eps);

    double poseTol = .5; // in meters
    double velTol = .5; // in m/s
    double dt = .1;      // in seconds
    double hDt = .05;

    double dv = limits_->maxAccel * dt;
    double dp = dv * .5 * dt;

    std::shared_ptr<Domain> env_ = std::make_shared<Domain>(
        map_, target_, limits_, eps, poseTol, velTol, dt, hDt);

    while (!env_->canPlan()) {
        ROS_WARN_THROTTLE(1, "PlannerDemo: waiting to plan....");

        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    std::shared_ptr<State> start_ = std::make_shared<State>();
    start_->numX = 0;
    start_->numY = 0;
    start_->numZ = 0;
    start_->numXVel = 0;
    start_->numYVel = 0;
    start_->numZVel = 0;
    start_->time = 0;

    std::shared_ptr<TargetPlanner> planner_ =
        std::make_shared<TargetPlanner>(env_, start_, eps);
    while (ros::ok()) {
        bool success = false;
        MotionPointStampedArray plan;
        MotionTrajectory traj;
        Path path;

        if (env_->canPlan()) {
            success = planner_->plan();
            if (success) {
                plan = planner_->getPlanTrajectory();

                traj.header.stamp = ros::Time::now();
                traj.header.frame_id = "world";
                traj.motion_points = plan.motion_points;

                point_pub.publish(traj);

                path.header.stamp = ros::Time::now();
                path.header.frame_id = "world";

                for (MotionPointStamped mp : plan.motion_points) {
                    PoseStamped pose;
                    pose.header = mp.header;
                    pose.pose = mp.motion_point.pose;
                    path.poses.push_back(pose);
                }
                plan_pub.publish(path);
            }

        } else {
            ROS_WARN_THROTTLE(1, "Planner Demo: waiting to plan...");
        }

        if (success) {
            point_pub.publish(plan);
            plan_pub.publish(path);

            ros::Duration(5).sleep();
            ros::spinOnce();

            if (curr_odom != nullptr) {
                std::shared_ptr<State> new_start = std::make_shared<State>();
                new_start->numX = static_cast<int>(std::floor(curr_odom->pose.pose.position.x/dp));
                new_start->numY = static_cast<int>(std::floor(curr_odom->pose.pose.position.y/dp));
                new_start->numZ = static_cast<int>(std::floor(curr_odom->pose.pose.position.z/dp));

                new_start->numXVel = static_cast<int>(std::floor(curr_odom->twist.twist.linear.x/dv));
                new_start->numYVel = static_cast<int>(std::floor(curr_odom->twist.twist.linear.x/dv));
                new_start->numZVel = static_cast<int>(std::floor(curr_odom->twist.twist.linear.x/dv));
                new_start->time = 0;

                planner_->setStart(new_start, eps);
            }
        }
        ros::spinOnce();
    }

    ros::spin();

    // All is good.
    return 0;
}
