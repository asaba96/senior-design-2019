#ifndef LIMITS_HPP
#define LIMITS_HPP

namespace Mbz2020Planner {
struct Limits {
    double maxX;
    double maxY;
    double maxZ;

    double minX;
    double minY;
    double minZ;

    double maxVel;
    double maxAccel;

    double maxTime;     // max time to plan out to
};
}

#endif // LIMITS_HPP
