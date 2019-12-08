#ifndef TARGET_PLANNER_HPP
#define TARGET_PLANNER_HPP

// system and ros headers
#include <ros/ros.h>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>

// data structure headers
#include <queue>
#include <unordered_map>

// message header
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbz2020_common/MotionPointStampedArray.h>

// planner headers
#include <target_planner/domain.hpp>

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using mbz2020_common::MotionPointStamped;

typedef mbz2020_common::MotionPointStampedArray PathTrajectory;

namespace Mbz2020Planner {

class TargetPlanner {
public:
    TargetPlanner(std::shared_ptr<Domain> env, std::shared_ptr<State> start, double eps);
    void setStart(std::shared_ptr<State> start, double eps);
    PathTrajectory getPlanTrajectory();
    bool plan();

    struct CompareSuccessors {
        bool operator()(const std::shared_ptr<const Successor>& a,
                        const std::shared_ptr<const Successor>& b) const {
            return (a->f) > (b->f);
        }
    };

private:
    bool replan();
    void resetOpenList(double eps);
    std::unordered_map<State, std::shared_ptr<Successor> > nodes;
    std::unordered_map<State, std::shared_ptr<Successor> > incons;

    std::priority_queue<std::shared_ptr<Successor>,
                        std::vector<std::shared_ptr<Successor> >,
                        CompareSuccessors>
    open_;

    int numSearches;
    double _eps;

    std::shared_ptr<Domain> _env;
    std::shared_ptr<Successor> goalNode;

    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> stopTime;
};
}
#endif // TARGET_PLANNER_HPP
