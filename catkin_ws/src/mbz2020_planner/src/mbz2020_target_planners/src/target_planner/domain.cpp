#include <target_planner/domain.hpp>

using namespace Mbz2020Planner;

Domain::Domain(std::shared_ptr<Map> map, std::shared_ptr<Target> target,
               std::shared_ptr<Limits> limits, double eps, double poseTol,
               double velTol, double dt, double hDt)
        : _target(target),
          _map(map),
          _limits(limits),
          _eps(eps),
          _poseTol(poseTol),
          _velTol(velTol),
          nodes(),
          closed(),
          open_() {
    _dt = dt;
    _dP = .5 * limits->maxAccel * _dt * _dt;
    _dV = limits->maxAccel * _dt;
    actionSpace = {-limits->maxAccel, 0, limits->maxAccel};
    maxAccelIn = 1;
    _Hdt = hDt;

    alreadySearched = false;
    SQRT2 = 1.4142135624;
    SQRT2_MINUS_ONE = 0.4142135624;
    DX = {-1, -1, -1, 0, 0, 1, 1, 1};
    DY = {-1, 0, 1, -1, 1, -1, 0, 1};
    COSTS = {SQRT2, 1, SQRT2, 1, 1, SQRT2, 1, SQRT2};
}

std::vector<std::shared_ptr<Successor> > Domain::getSuccessors(
    std::shared_ptr<Successor> currNode) {
    std::vector<std::shared_ptr<Successor> > resultList;

    std::shared_ptr<State> currState = currNode->state;

    for (int x = -maxAccelIn; x <= maxAccelIn; x++) {
        for (int y = -maxAccelIn; y <= maxAccelIn; y++) {
            for (int z = -maxAccelIn; z <= maxAccelIn; z++) {
                std::shared_ptr<State> nextCon = std::make_shared<State>();

                nextCon->numX = currState->numX + currState->numXVel * 2 + x;
                nextCon->numY = currState->numY + currState->numYVel * 2 + y;
                nextCon->numZ = currState->numZ + currState->numZVel * 2 + z;

                nextCon->numXVel = currState->numXVel + x;
                nextCon->numYVel = currState->numYVel + y;
                nextCon->numZVel = currState->numZVel + z;

                nextCon->time = currState->time + 1;

                GroundedState node = makeGroundedState(nextCon);

                if (isValid(node)) {
                    std::shared_ptr<Successor> nextSucc = std::make_shared<Successor>();
                    nextSucc->closed = 0;
                    nextSucc->done = false;
                    nextSucc->g = currNode->g + _dt;
                    nextSucc->v = std::numeric_limits<double>::infinity();
                    nextSucc->f = nextSucc->g + _eps * getHeuristic(nextCon);
                    nextSucc->state = nextCon;
                    nextSucc->parent = currNode;

                    resultList.push_back(nextSucc);
                }
            }
        }
    }
    return resultList;
}

double Domain::getHeuristic(std::shared_ptr<State> state) {
    GroundedState node = makeGroundedState(state);
    double h1 = getHeuristic(node);
    // double h2 = getHeuristicSearch(state, node);
    // ROS_ERROR_STREAM("H1: " << h1 << " H2: " << h2);
    return h1;
}

double Domain::getHeuristic(GroundedState node) {
    for (double t = node.time; t < _limits->maxTime; t += _Hdt) {
        // for each time step, estimate time to reach goal or each
        // direction and take min. if less than timestep, take timestep
        Odometry targetOdom = _target->getTargetAtTime(t);
        double tx =
            getHeuristicValue(node.x, node.xVel, targetOdom.pose.pose.position.x,
                              targetOdom.twist.twist.linear.x);
        double ty =
            getHeuristicValue(node.y, node.yVel, targetOdom.pose.pose.position.y,
                              targetOdom.twist.twist.linear.y);
        double tz =
            getHeuristicValue(node.z, node.zVel, targetOdom.pose.pose.position.z,
                              targetOdom.twist.twist.linear.z);

        double maxTime = std::max({tx, ty, tz});

        if (maxTime < (t - node.time)) {
            return (t - node.time);
        }
    }
    return std::numeric_limits<double>::infinity();
}

bool Domain::isGoal(std::shared_ptr<State> state) {
    GroundedState node = makeGroundedState(state);
    return isGoal(node);
}

bool Domain::isGoal(GroundedState node) {
    Odometry goalOdom = this->_target->getTargetAtTime(node.time);

    double poseDiff =
        std::sqrt(std::pow(goalOdom.pose.pose.position.x - node.x, 2) +
                  std::pow(goalOdom.pose.pose.position.y - node.y, 2) +
                  std::pow(goalOdom.pose.pose.position.z - node.z, 2));

    if (poseDiff > this->_poseTol) {
        return false;
    }

    double velDiff =
        std::sqrt(std::pow(goalOdom.twist.twist.linear.x - node.xVel, 2) +
                  std::pow(goalOdom.twist.twist.linear.y - node.yVel, 2) +
                  std::pow(goalOdom.twist.twist.linear.z - node.zVel, 2));

    return (velDiff > this->_velTol);
}

bool Domain::isValidWithPrints(GroundedState node) {
    // ROS_WARN_STREAM("IN BOUNDS: " << isInBounds(node)  << " IS FREE: " <<
    // _map->isFree(node.x, node.y, node.z, true));
    return isValid(node);
}

bool Domain::isValid(GroundedState node) {
    return (isInBounds(node) && this->_map->isFree(node.x, node.y, node.z));
}

bool Domain::canPlan() {
    // ROS_INFO_STREAM("CAN PLAN: HAS MAP: " << _map->hasMap() << " TARGET " << _target->hasTargetMessage());
    return (_map->hasMap() && _target->hasTargetMessage());
}

void Domain::setEps(int eps) {
    this->_eps = eps;
}

GroundedState Domain::makeGroundedState(std::shared_ptr<State> state) {
    GroundedState gs;
    gs.x = state->numX * _dP;
    gs.y = state->numY * _dP;
    gs.z = state->numZ * _dP;

    gs.xVel = state->numXVel * _dV;
    gs.yVel = state->numYVel * _dV;
    gs.zVel = state->numZVel * _dV;

    gs.time = state->time * _dt;
    return gs;
}

GroundedState Domain::makeGroundedState(std::shared_ptr<State> state,
                                        double xOff, double yOff, double zOff) {
    GroundedState gs;
    gs.x = state->numX * _dP + xOff;
    gs.y = state->numY * _dP + yOff;
    gs.z = state->numZ * _dP + zOff;

    gs.xVel = state->numXVel * _dV;
    gs.yVel = state->numYVel * _dV;
    gs.zVel = state->numZVel * _dV;

    gs.time = state->time * _dt;
    return gs;
}

bool Domain::isInBounds(GroundedState node) {
    if (node.x > this->_limits->maxX || node.y > this->_limits->maxY ||
        node.z > this->_limits->maxZ) {
        // node outside max map
        return false;
    }

    if (node.x < this->_limits->minX || node.y < this->_limits->minY ||
        node.z < this->_limits->minZ) {
        // node outside min map
        return false;
    }

    if (std::max({std::abs(node.xVel), std::abs(node.yVel),
                  std::abs(node.zVel)}) > this->_limits->maxVel) {
        // node exceeds velocity limits
        return false;
    }

    if (node.time > this->_limits->maxTime) {
        // node exceeds max planning time
        return false;
    }

    // all good
    return true;
}

double Domain::getHeuristicValue(double p1, double v1, double p2, double v2) {
    // assume max accel in one direction, then switching and max accel in
    // other direction
    double dX = p2 - p1;
    double dV = v2 - v1;
    double a = _limits->maxAccel;

    double b = v1 * v2 + 0.5 * std::pow(dV, 2) + a * dX;

    double t1 = (-2 * v1 - dV + 2 * std::sqrt(b)) / a;
    double t2 = (-2 * v1 - dV - 2 * std::sqrt(b)) / a;

    a = -a;
    b = v1 * v2 + 0.5 * std::pow(dV, 2) + a * dX;

    double t3 = (-2 * v1 - dV + 2 * std::sqrt(b)) / a;
    double t4 = (-2 * v1 - dV - 2 * std::sqrt(b)) / a;

    double tsw1 = 0.5 * (dV / a + t1);
    double tsw2 = 0.5 * (dV / a + t2);
    double tsw3 = 0.5 * (dV / -a + t3);
    double tsw4 = 0.5 * (dV / -a + t4);

    if (std::isnan(t1) || t1 < 0 || tsw1 < 0 || tsw1 > t1)
        t1 = std::numeric_limits<double>::infinity();
    if (std::isnan(t2) || t2 < 0 || tsw2 < 0 || tsw2 > t2)
        t2 = std::numeric_limits<double>::infinity();
    if (std::isnan(t3) || t3 < 0 || tsw3 < 0 || tsw3 > t3)
        t3 = std::numeric_limits<double>::infinity();
    if (std::isnan(t4) || t4 < 0 || tsw4 < 0 || tsw4 > t4)
        t4 = std::numeric_limits<double>::infinity();

    double result = std::min({t1, t2, t3, t4});

    if (std::isinf(result)) {
        throw std::runtime_error("Taregt Planner: bad heuristic");
    }

    return result;
}

double Domain::getHeuristicSearch(std::shared_ptr<State> currState,
                                  GroundedState groundedCurrState) {
    Odometry target = _target->getTargetAtTime(0);
    State stateToCheck;
    stateToCheck.numX = currState->numX;
    stateToCheck.numY = currState->numY;
    stateToCheck.numZ = 0;
    stateToCheck.numXVel = 0;
    stateToCheck.numYVel = 0;
    stateToCheck.numZVel = 0;
    stateToCheck.time = 0;

    double x = target.pose.pose.position.x;
    double y = target.pose.pose.position.y;

    if (closed.count(stateToCheck) > 0) {
        // already expanded this state
        ROS_WARN("ALREADY FOUND");
        return closed[*currState]->g;
    }
    // continue search
    if (!alreadySearched) {
        alreadySearched = true;
        // start planning from curr target pose
        std::shared_ptr<State> startState = std::make_shared<State>();
        startState->numX = static_cast<int>(std::floor(x / _dP));
        startState->numY = static_cast<int>(std::floor(y / _dP));
        startState->numZ = 0;
        startState->numXVel = 0;
        startState->numYVel = 0;
        startState->numZVel = 0;
        startState->time = 0;

        std::shared_ptr<Successor> start = std::make_shared<Successor>();
        start->closed = 0;
        start->done = false;
        start->f = getHeuristicForHSearch(groundedCurrState);
        start->g = 0.0;
        start->v = std::numeric_limits<double>::infinity();
        start->parent = nullptr;
        start->state = startState;
        open_.push(start);
    }

    while (ros::ok() && !open_.empty()) {
        std::shared_ptr<Successor> next = open_.top();
        open_.pop();

        if (next->done)
            continue;
        next->v = next->g;
        next->closed = 1;
        closed.insert({*next->state, next});

        if (next->state->numX == currState->numX &&
            next->state->numY == currState->numY) {
            // found node
            ROS_WARN("SUB GOAL!!!!!");
            return next->g;
        }

        if (next->state->numX == 0 && next->state->numY == 0) {
            ROS_ERROR("Found goal before node!");
        }

        for (std::shared_ptr<Successor> succ : getSuccessorsForHSearch(next)) {
            bool alreadySeen = nodes.count(*succ->state) > 0;
            if (!alreadySeen || nodes[*succ->state]->closed < 1) {
                if (!alreadySeen || succ->g < nodes[*succ->state]->g) {
                    // new node or already seen and has lower cost
                    open_.push(succ);
                    if (alreadySeen) {
                        nodes[*succ->state]->done = true;
                        nodes.erase(*succ->state);
                    }
                }
            } else {
                nodes[*succ->state]->g = succ->g;
            }
        }
    }
    return 0;
}

double Domain::getHeuristicForHSearch(GroundedState state) {
    // goal is start, which is at <0,0,0>
    double h = (SQRT2_MINUS_ONE * std::min(std::abs(state.x), std::abs(state.y)) +
                std::max(std::abs(state.x), std::abs(state.y)));
    return h;
}

std::vector<std::shared_ptr<Successor> > Domain::getSuccessorsForHSearch(
    std::shared_ptr<Successor> currNode) {
    std::vector<std::shared_ptr<Successor> > resultList;
    std::shared_ptr<State> currState = currNode->state;

    for (int i = 0; i < 8; i++) {
        int x = DX[i];
        int y = DY[i];
        double cost = COSTS[i];

        std::shared_ptr<State> nextCon = std::make_shared<State>();

        nextCon->numX = currState->numX + x;
        nextCon->numY = currState->numY + y;
        nextCon->numZ = 0;

        nextCon->numXVel = 0;
        nextCon->numYVel = 0;
        nextCon->numZVel = 0;

        nextCon->time = 0;

        GroundedState node = makeGroundedState(nextCon);

        if (isValidWithPrints(node)) {
            std::shared_ptr<Successor> nextSucc = std::make_shared<Successor>();

            nextSucc->closed = 0;
            nextSucc->done = false;
            nextSucc->g = currNode->g + cost;
            nextSucc->v = std::numeric_limits<double>::infinity();
            nextSucc->f = nextSucc->g + 10 * getHeuristicForHSearch(node);
            nextSucc->state = nextCon;
            nextSucc->parent = currNode;

            resultList.push_back(nextSucc);
        }
    }

    return resultList;
}
