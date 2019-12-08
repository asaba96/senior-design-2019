#ifndef MAP_HPP
#define MAP_HPP

// system and ros headers
#include <ros/ros.h>

// message headers
#include <mbz2020_common/MapGrid.h>
#include <geometry_msgs/Point.h>

using mbz2020_common::MapGrid;
using geometry_msgs::Point;

namespace Mbz2020Planner {

class Map {
public:
    Map(ros::NodeHandle& nh, double resolution);
    bool isFree(double x, double y, double z, bool print = false);
    bool hasMap();
private:
    void mapCallback(const mbz2020_common::MapGrid::ConstPtr& msg);

    ros::Subscriber map_sub;

    mbz2020_common::MapGrid _map;

    double _resolution;
    bool _has_map;
    bool _check_obstacles;
};
}
#endif // MAP_HPP
