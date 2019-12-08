#include <trajectory_generator/trajectory_generator_nonlinear.hpp>

using namespace TrajectoryGenerator;

// Main entry point for the motion planner
int main(int argc, char** argv)
{
    // Required by ROS before calling many functions
    ros::init(argc, argv, "nonlinear_trajectory_generator");

    ROS_INFO("TrajectoryPlanner begin");

    // Create a node handle for the node
    ros::NodeHandle nh;

    // Wait for a valid time in case we are using simulated time (not wall time)
    while (ros::ok() && ros::Time::now() == ros::Time(0)) {
        // wait
        ros::spinOnce();
        ros::Duration(.005).sleep();
    }

    TrajectoryGeneratorNonLinear tgl(nh);

    ros::spin();

    // All is good.
    return 0;
}
