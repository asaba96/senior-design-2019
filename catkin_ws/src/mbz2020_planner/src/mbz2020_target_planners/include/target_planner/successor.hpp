#ifndef SUCESSOR_HPP
#define SUCESSOR_HPP

// system headers
#include <memory>
#include <iostream>

// planner headers
#include <target_planner/state.hpp>

namespace Mbz2020Planner {
struct Successor {
    int closed;
    bool done;
    double g = std::numeric_limits<double>::quiet_NaN();
    double v = std::numeric_limits<double>::quiet_NaN();
    double f = std::numeric_limits<double>::quiet_NaN();
    std::shared_ptr<State> state;
    std::shared_ptr<Successor> parent;
};
}

#endif // SUCESSOR_HPP
