#include <target_planner/target_planner.hpp>

namespace Mbz2020Planner {

TargetPlanner::TargetPlanner(std::shared_ptr<Domain> env,
                             std::shared_ptr<State> start, double eps)
        : open_(), _env(env) {
    numSearches = 0;
    goalNode = nullptr;
    setStart(start, eps);
    _eps = eps;
}

void TargetPlanner::setStart(std::shared_ptr<State> start, double eps) {
    std::shared_ptr<Successor> startNode = std::make_shared<Successor>();
    GroundedState node = _env->makeGroundedState(start);
    startNode->closed = 0;
    startNode->done = false;
    startNode->g = 0.0;
    startNode->v = std::numeric_limits<double>::infinity();
    startNode->f = eps * _env->getHeuristic(node);
    startNode->state = start;
    startNode->parent = nullptr;

    nodes.insert({*start, startNode});
    open_.push(startNode);
}

PathTrajectory TargetPlanner::getPlanTrajectory() {
    PathTrajectory p;
    double totalCost = 0;

    std::vector<MotionPointStamped> planVector;

    if (this->goalNode == nullptr) {
        ROS_ERROR("PathPlanner: Goal Node is null");
        return p;
    }

    std::shared_ptr<const Successor> parent = this->goalNode;
    totalCost = parent->g;

    while (parent != nullptr) {
        GroundedState gs = _env->makeGroundedState(parent->state);
        MotionPointStamped pt = gs.toMotionPoint();
        planVector.push_back(pt);
        parent = parent->parent;
    }

    ROS_INFO_STREAM("TargetPathPlanner: total plan cost: " << totalCost);

    std::reverse(planVector.begin(), planVector.end());

    p.motion_points = planVector;
    return p;
}

bool TargetPlanner::plan() {
    this->startTime = std::chrono::high_resolution_clock::now();
    this->stopTime = startTime + std::chrono::milliseconds(500);

    this->numSearches = 1;

    double eps = this->_eps;
    bool result = false;

    while (eps > 1.0) {
        eps = eps - .5;
        eps = std::max(eps, 1.0);

        ROS_INFO("TargetPlanner: search %d with EPS %f", numSearches, eps);

        this->resetOpenList(eps);
        this->_env->setEps(eps);

        result = replan() || result;

        if (std::chrono::high_resolution_clock::now() > this->stopTime) {
            ROS_WARN("TargetPlanner: ran out of time with %d searches, ending at epsilon %f", numSearches, eps);
            break;
        }

        numSearches++;
    }
    return result;
}

bool TargetPlanner::replan() {
    bool success = false;

    if (!this->_env->canPlan()) {
        ROS_ERROR("TargetPlanner: cannot plan yet");
        return success;
    }

    // ROS_INFO("TargetPlanner: Start of search %d", numSearches);
    std::chrono::time_point<std::chrono::high_resolution_clock> replanStartTime =
        std::chrono::high_resolution_clock::now();

    int numExpand = 0;

    while (ros::ok() && !open_.empty()) {
        if (std::chrono::high_resolution_clock::now() > this->stopTime) {
            ROS_WARN("TargetPlanner: ran out of time with %d searches",
           numSearches);
            return false;
        }

        if (goalNode != nullptr && goalNode->g <= open_.top()->f) {
            // done this round
            const auto rePlanEnd = std::chrono::high_resolution_clock::now();
            ROS_INFO_STREAM("TargetPlanner: done replanning");
            ROS_INFO_STREAM("TargetPlanner: num nodes expanded: " << numExpand);
            ROS_INFO_STREAM("TargetPlanner: re-planning time: "
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                rePlanEnd - replanStartTime)
                            .count()
                            << "ms.");
            ROS_INFO_STREAM("TargetPlanner: G of GoalNode: "
                            << goalNode->g << " open V: " << open_.top()->f);

            return true;
        }

        std::shared_ptr<Successor> next = open_.top();
        open_.pop();

        if (next->done) {
            // done
            continue;
        }

        numExpand++;
        next->v = next->g;
        next->closed = numSearches;

        if (_env->isGoal(next->state) &&
            (goalNode == nullptr || goalNode->g > next->g)) {
            const auto rePlanEnd = std::chrono::high_resolution_clock::now();
            ROS_INFO_STREAM("Targetlanner: Goal found!");
            ROS_INFO_STREAM("TargetPLanner: num nodes expanded: " << numExpand);
            ROS_INFO_STREAM("TargetPlanner: planning time: "
                            << std::chrono::duration_cast<std::chrono::milliseconds>(
                                rePlanEnd - replanStartTime)
                            .count()
                            << "ms.");

            this->goalNode = next;
            return true;
        }

        // get all successors
        for (std::shared_ptr<Successor> newSucc : _env->getSuccessors(next)) {
            bool alreadySeen = (nodes.count(*newSucc->state) != 0);

            if (!alreadySeen || nodes[*newSucc->state]->closed < numSearches) {
                if (!alreadySeen || newSucc->g < nodes[*newSucc->state]->g) {
                    // new node or already seen and has lower cost
                    open_.push(newSucc);
                    if (alreadySeen) {
                        nodes[*newSucc->state]->done = true;
                        nodes.erase(*newSucc->state);
                    }
                    nodes.insert({*newSucc->state, newSucc});
                }
            } else {
                // already closed
                nodes[*newSucc->state]->g = newSucc->g;
                if (incons.count(*newSucc->state) == 0) {
                    incons.insert({*newSucc->state, nodes[*newSucc->state]});
                }
            }
        }
    }

    ROS_ERROR_STREAM(
        "TargetPlanner: NO PATH FOUND!! NODES EXPANDED: " << numExpand);
    return false;
}

void TargetPlanner::resetOpenList(double eps) {
    std::priority_queue<std::shared_ptr<Successor>,
                        std::vector<std::shared_ptr<Successor> >,
                        CompareSuccessors>
    newQueue;

    while (ros::ok() && !open_.empty()) {
        std::shared_ptr<Successor> next = open_.top();
        open_.pop();

        if (next->done)
            continue;

        next->f = next->g + eps * _env->getHeuristic(next->state);
        newQueue.push(next);
    }

    for (auto& set : incons) {
        std::shared_ptr<Successor> node = set.second;
        node->f = node->g + eps * _env->getHeuristic(node->state);
        newQueue.push(node);
    }

    incons.clear();
    open_.swap(newQueue);
}
}  // namespace Mbz2020Planner
