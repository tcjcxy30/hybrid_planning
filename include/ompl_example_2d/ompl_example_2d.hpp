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

#include <ompl-1.6/ompl/base/DiscreteMotionValidator.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl-1.6/ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl-1.6/ompl/geometric/SimpleSetup.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl-1.6/ompl/geometric/planners/rrt/RRTstar.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

namespace ompl_example_2d {

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

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::StateSpace> space;

    // get map information
    void getBounds(double& min_x, double& max_x, double& min_y, double& max_y,
                   const nav_msgs::msg::OccupancyGrid& ogm);

    /// extract path
    nav_msgs::msg::Path extractPath(ompl::geometric::SimpleSetup& ss);
};

}   // namespace ompl_example_2d
