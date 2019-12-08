#ifndef DOMAIN_HPP
#define DOMAIN_HPP

// system and ros headers
#include <ros/ros.h>
#include <memory>
#include <queue>
#include <unordered_map>

// message headers
#include <nav_msgs/Odometry.h>

// planner headers
#include <target_planner/target.hpp>
#include <target_planner/map.hpp>
#include <target_planner/successor.hpp>
#include <target_planner/limits.hpp>

using nav_msgs::Odometry;

namespace Mbz2020Planner {

class Domain {
public:
    Domain(std::shared_ptr<Map> map,
           std::shared_ptr<Target> target,
           std::shared_ptr<Limits> limits,
           double eps,
           double poseTol,
           double velTol,
           double dt,
           double hDt);

    std::vector<std::shared_ptr<Successor> >
    getSuccessors(std::shared_ptr<Successor> currState);

    double getHeuristic(std::shared_ptr<State> state);
    double getHeuristic(GroundedState node);

    bool isGoal(std::shared_ptr<State> node);
    bool isGoal(GroundedState node);

    bool isValidWithPrints(GroundedState node);
    bool isValid(GroundedState node);
    bool canPlan();
    void setEps(int eps);

    GroundedState makeGroundedState(std::shared_ptr<State> state);
    GroundedState makeGroundedState(std::shared_ptr<State> state,
                                    double xOff,
                                    double yOff,
                                    double zOff);
private:
    bool isInBounds(GroundedState node);
    double getHeuristicValue(double p1, double v1, double p2, double v2);
    double getHeuristicSearch(std::shared_ptr<State> currState, GroundedState gs);
    double getHeuristicForHSearch(GroundedState state);
    std::vector<std::shared_ptr<Successor> >
    getSuccessorsForHSearch(std::shared_ptr<Successor> currState);

    std::shared_ptr<Target> _target;
    std::shared_ptr<Map> _map;
    std::shared_ptr<Limits> _limits;

    std::array<double, 3> actionSpace;
    double _dt;      // discretization of time space
    double _dP;
    double _dV;

    double _Hdt;     // dt used when finding interception heuristic time
    double _eps;     // heuristic inflation factor
    double _poseTol;     // tolerance around goal pose
    double _velTol;     // tolerance around goal vel
    int maxAccelIn;

    // needed for subsearch
    bool alreadySearched;
    double SQRT2;
    double SQRT2_MINUS_ONE;
    std::array<int, 8> DX;
    std::array<int, 8> DY;
    std::array<double, 8> COSTS;

    struct CompareNodes {
        bool operator()(const std::shared_ptr<const Successor>& a,
                        const std::shared_ptr<const Successor>& b) const {
            return (a->f) > (b->f);
        }
    };

    std::unordered_map<State, std::shared_ptr<Successor> > nodes;
    std::unordered_map<State, std::shared_ptr<Successor> > closed;

    std::priority_queue<std::shared_ptr<Successor>,
                        std::vector<std::shared_ptr<Successor> >,
                        CompareNodes>
    open_;
};
}
#endif // DOMAIN_HPP
