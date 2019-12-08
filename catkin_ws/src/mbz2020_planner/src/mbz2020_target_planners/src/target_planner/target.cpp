#include <target_planner/target.hpp>

namespace Mbz2020Planner {

Target::Target(MotionTrajectory trajectory) {
    _isStatic = true;
    _trajectory = trajectory;
    _hasTargetMsg = true;
}

Target::Target(ros::NodeHandle& nh)
        : targetSub(nh.subscribe("/target", 10, &Target::targetCallback, this)) {
    _isStatic = false;
    _hasTargetMsg = false;
}

Odometry Target::getTargetAtTime(double time) {
    if (!_hasTargetMsg) {
        throw std::runtime_error(
                  "TargetPlanner: no target msg available and getTargetAtTime called");

    }

    ros::Time rosTimeTarget = ros::Time::now() + ros::Duration(time);

    Odometry finalOdom;
    finalOdom.header.stamp = rosTimeTarget;

    if (_isStatic) {
        // whole trajectory known and is not updated via callback
        // and assumed to start at time = 0

        double numStepsRaw = time / _trajectory.dt;

        int numSteps = static_cast<int>(std::floor(numStepsRaw));
        int numPoints = _trajectory.motion_points.size();
        int index = -1;

        if (numPoints < numSteps) {
            // loop back over start
            index = (numSteps % numPoints) - 1;
        } else {
            index = numSteps - 1;
        }

        int index2 = index;

        if (index == (numPoints - 1)) {
            index2 = 0;
        } else {
            index2 = index + 1;
        }

        double stepAmt = numStepsRaw - numSteps;

        Point poseStart = _trajectory.motion_points[index].motion_point.pose.position;
        Point poseEnd = _trajectory.motion_points[index2].motion_point.pose.position;

        Vector3 velStart = _trajectory.motion_points[index].motion_point.twist.linear;
        Vector3 velEnd = _trajectory.motion_points[index2].motion_point.twist.linear;

        Point finalPose;
        Vector3 finalVel;

        finalPose.x = ((poseEnd.x - poseStart.x) * stepAmt + poseStart.x);
        finalPose.y = ((poseEnd.y - poseStart.y) * stepAmt + poseStart.y);
        finalPose.z = ((poseEnd.z - poseStart.z) * stepAmt + poseStart.z);

        finalVel.x = ((velEnd.y - velStart.x) * stepAmt + velStart.x);
        finalVel.y = ((velEnd.y - velStart.x) * stepAmt + velStart.x);
        finalVel.z = ((velEnd.y - velStart.x) * stepAmt + velStart.x);

        finalOdom.pose.pose.position = finalPose;
        finalOdom.twist.twist.linear = finalVel;
    }
    else {
        if (currOdom.header.stamp > rosTimeTarget) {
            ROS_ERROR_STREAM("header stamp: " << currOdom.header.stamp << " ROS TIME TARGET " << rosTimeTarget);
            ROS_ERROR("TargetPlanner: Looking up target pose in past");
            throw std::runtime_error("TargetPlanner: bad timestamps");
        }

        // assume maintains same velocity and do linear extrapolation
        double secondsRaw = (rosTimeTarget - currOdom.header.stamp).toSec();

        finalOdom.pose.pose.position.x = currOdom.pose.pose.position.x +
                                         secondsRaw * currOdom.twist.twist.linear.x;
        finalOdom.pose.pose.position.y = currOdom.pose.pose.position.y +
                                         secondsRaw * currOdom.twist.twist.linear.y;
        finalOdom.pose.pose.position.z = currOdom.pose.pose.position.z +
                                         secondsRaw * currOdom.twist.twist.linear.z;

        finalOdom.twist = currOdom.twist;

        finalOdom.child_frame_id = currOdom.child_frame_id;
        finalOdom.header = currOdom.header;
    }

    return finalOdom;
}

bool Target::hasTargetMessage() {
    return _hasTargetMsg;
}

void Target::targetCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    this->_hasTargetMsg = true;
    this->currOdom = *msg;
}

}  // namespace Mbz2020Planner
