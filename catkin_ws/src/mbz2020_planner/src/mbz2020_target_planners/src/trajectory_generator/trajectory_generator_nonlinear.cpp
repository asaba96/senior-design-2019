#include <trajectory_generator/trajectory_generator_nonlinear.hpp>

namespace TrajectoryGenerator {

void TrajectoryGeneratorNonLinear::convertToMotionTrajectory(mbz2020_common::MotionTrajectory& msg,
                                                             const mav_msgs::EigenTrajectoryPoint::Vector states) {
    auto start_time = ros::Time::now();

    for (auto state : states) {
        mbz2020_common::MotionPointStamped mps;
        mps.header.stamp = start_time + ros::Duration(static_cast<double>(state.time_from_start_ns) / 1000000000.0);
        mps.motion_point.pose.position.x = state.position_W.x();
        mps.motion_point.pose.position.y = state.position_W.y();
        mps.motion_point.pose.position.z = state.position_W.z();

        mps.motion_point.pose.orientation.x = state.orientation_W_B.x();
        mps.motion_point.pose.orientation.y = state.orientation_W_B.y();
        mps.motion_point.pose.orientation.z = state.orientation_W_B.z();
        mps.motion_point.pose.orientation.w = state.orientation_W_B.w();

        mps.motion_point.twist.linear.x = state.velocity_W.x();
        mps.motion_point.twist.linear.y = state.velocity_W.y();
        mps.motion_point.twist.linear.z = state.velocity_W.z();

        mps.motion_point.twist.angular.x = state.angular_velocity_W.x();
        mps.motion_point.twist.angular.y = state.angular_velocity_W.y();
        mps.motion_point.twist.angular.z = state.angular_velocity_W.z();

        mps.motion_point.accel.linear.x = state.acceleration_W.x();
        mps.motion_point.accel.linear.y = state.acceleration_W.y();
        mps.motion_point.accel.linear.z = state.acceleration_W.z();

        mps.motion_point.accel.angular.x = state.angular_acceleration_W.x();
        mps.motion_point.accel.angular.y = state.angular_acceleration_W.y();
        mps.motion_point.accel.angular.z = state.angular_acceleration_W.z();

        msg.motion_points.push_back(mps);
    }
}

void TrajectoryGeneratorNonLinear::requestCallback(const mbz2020_target_planners::PlanRequestGoalConstPtr& goal_) {
    mbz2020_target_planners::PlanRequestResult result_;

    try {
        ROS_DEBUG("TrajectoryGeneratorNonlinear: New goal accepted by planner");

        mav_trajectory_generation::Vertex::Vector vertices;
        mav_trajectory_generation::Vertex start(dimension), end(dimension);

        int total_points = goal_->waypoints.size();

        if (total_points < 2) {
            ROS_ERROR("TrajectoryGeneratorNonLinear: ERROR: Too few waypoints provided to planner");
            result_.success = false;
            server.setAborted(result_);
        } else {
            geometry_msgs::Point start_pose = goal_->waypoints[0];
            geometry_msgs::Point end_pose = goal_->waypoints[total_points - 1];

            // setup start and end points
            start.makeStartOrEnd(Eigen::Vector3d(start_pose.x, start_pose.y, start_pose.z), derivative_to_optimize);
            vertices.push_back(start);

            // add all middle points
            for (int i = 1; i < total_points - 1; i++) {
                geometry_msgs::Point vert = goal_->waypoints[i];
                mav_trajectory_generation::Vertex middle(dimension);
                middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                                     Eigen::Vector3d(vert.x,vert.y,vert.z));
                vertices.push_back(middle);
            }

            end.makeStartOrEnd(Eigen::Vector3d(end_pose.x, end_pose.y, end_pose.z), derivative_to_optimize);
            vertices.push_back(end);

            std::vector<double> segment_times;
            segment_times = estimateSegmentTimes(vertices, v_max, a_max);

            // default parameters, should work okay
            mav_trajectory_generation::NonlinearOptimizationParameters parameters;
            parameters.max_iterations = 1000;
            parameters.f_rel = 0.05;
            parameters.x_rel = 0.1;
            parameters.time_penalty = 500.0;
            parameters.initial_stepsize_rel = 0.1;
            parameters.inequality_constraint_tolerance = 0.1;

            const int N = 10;
            mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
            opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
            opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
            opt.optimize();

            // convert to trajectory
            mav_trajectory_generation::Trajectory trajectory;
            opt.getTrajectory(&trajectory);

            // sample whole trajectory
            mav_msgs::EigenTrajectoryPoint::Vector states;
            bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

            // convert to motion trajectory
            mbz2020_common::MotionTrajectory traj_msg;
            convertToMotionTrajectory(traj_msg, states);

            // visualize in RVIZ
            visualization_msgs::MarkerArray markers;
            double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
            std::string frame_id = "world";

            mav_trajectory_generation::drawMavSampledTrajectory(states, distance, frame_id, &markers);

            marker_pub.publish(markers);

            result_.success = success;
            result_.plan = traj_msg;

            server.setSucceeded(result_);
            ROS_DEBUG("TrajectoryGeneratorNonLinear: Finished plan request.");
        }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("TrajectoryGeneratorNonLinear: error generating trajectory: " << e.what());
        result_.success = false;
        server.setSucceeded(result_);
    }
}

TrajectoryGeneratorNonLinear::TrajectoryGeneratorNonLinear(ros::NodeHandle& nh) :
        server(nh, "nonlinear_trajectory_request",
               boost::bind(&TrajectoryGeneratorNonLinear::requestCallback, this, _1), false)
{
    // visualization publisher
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("nonlinear_trajectory_markers", 1000);

    server.start();
}
}
