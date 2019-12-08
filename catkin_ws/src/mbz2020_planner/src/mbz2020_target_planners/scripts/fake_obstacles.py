#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

from mbz2020_common.msg import MapGrid

if __name__ == "__main__":
    rospy.init_node("fake_obstalces")

    map_pub = rospy.Publisher("/fake_map", OccupancyGrid, queue_size=10)
    grid_pub = rospy.Publisher("/fake_grid", MapGrid, queue_size=10)

    x_meters = 40
    y_meters = 40

    resolution = 0.1  # 10 cm

    numX = int(x_meters / resolution)
    numY = int(y_meters / resolution)

    x_bounds_lower = 25 / resolution
    x_bounds_upper = 26 / resolution

    y_bounds_free = 24 / resolution
    y_bounds_free_upper = 27 / resolution

    map_im = np.zeros(numX * numY, dtype=int)
    map_grid = np.zeros(numX * numY, dtype=int)

    for y in range(0, numY):
        for x in range(0, numX):
            index = numY * y + x
            if x_bounds_lower <= x <= x_bounds_upper:
                if y <= y_bounds_free or y >= y_bounds_free_upper:
                    # inside obstacle
                    map_im[index] = 100
                    map_grid[index] = 200

    map_ = map_im.tolist()
    grid_ = map_grid.tolist()

    grid_msg = MapGrid()
    grid_msg.header.frame_id = "world"
    grid_msg.data = map_
    grid_msg.info.height = numY
    grid_msg.info.width = numX
    grid_msg.info.resolution = resolution

    grid_msg.info.origin.position.x = -x_meters / 2
    grid_msg.info.origin.position.y = -y_meters / 2

    msg_ = OccupancyGrid()
    msg_.header.frame_id = "world"
    msg_.data = map_
    msg_.info.height = numY
    msg_.info.width = numX
    msg_.info.resolution = resolution

    msg_.info.origin.position.x = -x_meters / 2
    msg_.info.origin.position.y = -y_meters / 2

    rate_ = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg_.header.stamp = rospy.Time.now()
        map_pub.publish(msg_)
        grid_pub.publish(grid_msg)
        rate_.sleep()
