#include "ompl_example_2d/ompl_example_2d.hpp"
#include "collision_checker.h"
#include <rclcpp/rclcpp.hpp>

// STL
#include <limits>
#include <math.h>
#include <string>

using namespace std;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_example_2d {

// collision checker
std::shared_ptr<CollisionChecker> collision_checker;

Planner2D::Planner2D(void)
{
    std::cout << "planner 2D started\n";
    // configure();
}

Planner2D::~Planner2D() {}

/// extract path
nav_msgs::msg::Path Planner2D::extractPath(ompl::geometric::SimpleSetup& ss)
{
    nav_msgs::msg::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    // get the obtained path
    auto path = ss.getSolutionPath();
    for (unsigned int i = 0; i < path.getStateCount(); ++i) {
        // get state
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get state...");
        const ob::State* state = path.getState(i);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set coordX...");
        // Convert to RealVectorStateSpace
        const ob::RealVectorStateSpace::StateType* real_state =
            static_cast<const ob::RealVectorStateSpace::StateType*>(state);
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.pose.position.x = real_state->values[0];
        poseMsg.pose.position.y = real_state->values[1];
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "something wrong...");
        poseMsg.pose.position.z    = 0.01;
        poseMsg.pose.orientation.w = 1.0;
        poseMsg.pose.orientation.x = 0.0;
        poseMsg.pose.orientation.y = 0.0;
        poseMsg.pose.orientation.z = 0.0;
        poseMsg.header.frame_id    = "/map";
        poseMsg.header.stamp       = rclcpp::Clock().now();
        // ... and add the pose to the path
        plannedPath.poses.push_back(poseMsg);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "plot add once...");
    }
    std::cout << "planned path size: " << plannedPath.poses.size() << "\n";
    return plannedPath;
}

/*!
 * plan path
 */
nav_msgs::msg::Path Planner2D::planPath(nav_msgs::msg::OccupancyGrid& globalMap)
{
    space = std::make_shared<ob::RealVectorStateSpace>(2u);

    // init planner
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "initiallizing ompl_example_2d planner...");
    double min_x(0.0), max_x(0.0), min_y(0.0), max_y(0.0);
    getBounds(min_x, max_x, min_y, max_y, globalMap);

    // 2. RealVectorBounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, min_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(0, max_x);
    bounds.setHigh(1, max_y);
    // 3. set StateSpace bounds
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // search space information
    // 4. simpleSetup
    auto ss = std::make_shared<og::SimpleSetup>(space);
    auto si = ss->getSpaceInformation();

    // 5. custom setStateValidityChecker
    // custom State Validity Checker class need to be implemented which uses
    // occupancy grid maps for collison checking
    collision_checker = std::make_shared<CollisionChecker>(globalMap);

    ss->setStateValidityChecker(
        [this](const ob::State* state) { return collision_checker->isValid(state); });

    // 6. set the start and goal states
    auto start_state  = std::make_shared<ob::ScopedState<>>(space);
    (*start_state)[0] = 0.0;
    (*start_state)[1] = 0.0;
    auto goal_state   = std::make_shared<ob::ScopedState<>>(space);
    (*goal_state)[0]  = 5.0;
    (*goal_state)[1]  = 5.0;

    // 7. set the start and goal states
    ss->setStartAndGoalStates(*start_state, *goal_state);
    // objective
    auto optimization_objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    ss->setOptimizationObjective(optimization_objective);

    // termination condition
    std::shared_ptr<ob::PlannerTerminationCondition> ptc;
    double                                           planning_duration(2.0);
    ptc = std::make_shared<ob::PlannerTerminationCondition>(
        ob::timedPlannerTerminationCondition(planning_duration, 0.01));

    ss->setup();
    // rrtstar
    ss->setPlanner(ob::PlannerPtr(std::make_shared<og::RRTstar>(si)));

    // 8. solve
    auto                solved = ss->solve(*ptc);
    nav_msgs::msg::Path plannedPath;
    if (solved) {   // if cussess
        // get the planned path
        plannedPath = extractPath(*ss);
    }
    std::cout << "path planned\n";
    return plannedPath;
}

void Planner2D::getBounds(double& min_x, double& max_x, double& min_y, double& max_y,
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

    return;
}

}   // namespace ompl_example_2d
