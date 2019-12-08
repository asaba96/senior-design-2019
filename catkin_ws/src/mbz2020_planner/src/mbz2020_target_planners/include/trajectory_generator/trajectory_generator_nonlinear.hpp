#ifndef TRAJECTORY_GENERATOR_NON_LINEAR_HPP
#define TRAJECTORY_GENERATOR_NON_LINEAR_HPP

// ROS headers
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

// planner headers
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

// message headers
#include <geometry_msgs/Point.h>
#include <mbz2020_target_planners/PlanRequestAction.h>
#include <mbz2020_common/MotionTrajectory.h>
#include <mbz2020_common/MotionPointStamped.h>


typedef actionlib::SimpleActionServer<mbz2020_target_planners::PlanRequestAction> Server;

namespace TrajectoryGenerator {
class TrajectoryGeneratorNonLinear
{
private:
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

    const double v_max = 12.0;
    const double a_max = 4.0;
    const double sampling_interval = .01;

    Server server;
    ros::Publisher marker_pub;

    void convertToMotionTrajectory(mbz2020_common::MotionTrajectory& msg,
                                   const mav_msgs::EigenTrajectoryPoint::Vector states);

    void requestCallback(const mbz2020_target_planners::PlanRequestGoalConstPtr& goal_);

public:
    TrajectoryGeneratorNonLinear(ros::NodeHandle& nh);

    ~TrajectoryGeneratorNonLinear(void) {
    }

};
}

#endif // TRAJECTORY_GENERATOR_NON_LINEAR_HPP
