#pragma once

#define MAP_UTILS_DEBUG 0

#include "nav_msgs/msg/occupancy_grid.hpp"
#include <math.h>

/**
 * @brief OccupancyGrid data constants
 */
static constexpr int8_t OCC_GRID_UNKNOWN  = -1;
static constexpr int8_t OCC_GRID_FREE     = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;

namespace map_utils {
/**
 * @brief Generate realvector bounds of an 2D occupancy grid map
 * @param min_x Lower map bound in x-axis
 * @param max_x Higher map bound in x-axis
 * @param min_y Lower map bound in y-axis
 * @param max_y Higher map bound in y-axis
 * @return true if can generate bounds
 */
inline bool getBounds(double& min_x, double& max_x, double& min_y, double& max_y,
                      const nav_msgs::msg::OccupancyGrid& ogm)
{
    // extract map parameters
    unsigned int cells_size_x = ogm.info.width;
    unsigned int cells_size_y = ogm.info.height;
    double       resolution   = static_cast<double>(ogm.info.resolution);
    double       origin_x     = ogm.info.origin.position.x;
    double       origin_y     = ogm.info.origin.position.y;

    double map_size_x = cells_size_x * resolution;
    double map_size_y = cells_size_y * resolution;

    min_x = origin_x;
    min_y = origin_y;
    max_x = map_size_x - fabs(origin_x);
    max_y = map_size_y - fabs(origin_y);

#if MAP_UTILS_DEBUG
    std::cout << "[DEBUG] [map_utils] Map size in meters: " << map_size_x << " X " << map_size_y
              << std::endl;
    std::cout << "[DEBUG] [map_utils] Map bounds (lower-left): " << min_x << ", " << min_y
              << std::endl;
    std::cout << "[DEBUG] [map_utils] Map bounds (upper-right): " << max_x << ", " << max_y
              << std::endl;
#endif

    return true;
}

}   // namespace map_utils
