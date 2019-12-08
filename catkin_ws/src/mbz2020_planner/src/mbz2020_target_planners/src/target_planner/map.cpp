#include <target_planner/map.hpp>

using namespace Mbz2020Planner;

Map::Map(ros::NodeHandle& nh, double resolution)
        : _resolution(resolution),
          map_sub(nh.subscribe("/fake_grid", 10, &Map::mapCallback, this)) {
    _has_map = false;
    nh.getParam("/check_obstacles", _check_obstacles);
}

bool Map::isFree(double x, double y, double z, bool print) {
    if (!_check_obstacles) {
        return true;
    } else {
        int mapWidth = _map.info.width;
        double mapRes = _map.info.resolution;

        double originX = std::abs(_map.info.origin.position.x);
        double originY = std::abs(_map.info.origin.position.y);

        int xMap = static_cast<int>((x + originX) / mapRes);
        int yMap = static_cast<int>((y + originY) / mapRes);
        unsigned long zMap = static_cast<unsigned long>(std::abs(z / mapRes));

        if (xMap < 0 || yMap < 0) {
            ROS_ERROR_STREAM("TargetPlanner: X: " << xMap << " and Y: " << yMap
                                                  << " cell in isFree is negative");
            return false;
        }

        if (xMap > mapWidth || yMap > mapWidth) {
            ROS_ERROR_STREAM("TargetPlanner: X or Y cells are outside map. X = "
                             << xMap << " Y = " << yMap);
            return false;
        }

        // convert to row major order
        int index = static_cast<int>(yMap * mapWidth + xMap);

        if (print) {
            ROS_WARN_STREAM("z: " << z << " ZMAP: " << zMap);
        }

        if (zMap == 0 && _map.data[index] == 0) {
            return true;
        }

        return (_map.data[index] < zMap);
    }
}

bool Map::hasMap() {
    return _has_map;
}

void Map::mapCallback(const mbz2020_common::MapGrid::ConstPtr& msg) {
    this->_has_map = true;
    this->_map = *msg;
}
