/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *  Updated on: April 8, 2023
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// ROS
#include "geometry_msgs/msg/pose_stamped.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

// Boost
#include <Eigen/Eigen>
#include <boost/thread.hpp>
// standard
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

#include <vector>

namespace hybrid_planner {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:
    /*!
     * Constructor.
     */
    Planner2D(void);

    /*!
     * Destructor.
     */
    virtual ~Planner2D();

    /*!
     * plan path
     */
    nav_msgs::msg::Path planPath(nav_msgs::msg::OccupancyGrid& globalMap);

private:
    /// problem dim
    int dim;

    /// max step length
    double maxStepLength;

    /// extract path
    nav_msgs::msg::Path extractPath(std::vector<Eigen::Vector3d>& pts);

    // plot graph
};

}   // namespace hybrid_planner
