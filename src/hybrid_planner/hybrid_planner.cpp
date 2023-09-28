#include "hybrid_planner/hybrid_planner.hpp"
#include "hybrid_planner/topo_prm.h"
#include <rclcpp/rclcpp.hpp>

// STL
#include <limits>
#include <math.h>
#include <string>

using namespace std;
namespace hybrid_planner {

// occupancy map used for planning
nav_msgs::msg::OccupancyGrid occupancyMap;

Planner2D::Planner2D(void)
{
    std::cout << "planner 2D started\n";
    // configure();
}

Planner2D::~Planner2D() {}

/// extract path
nav_msgs::msg::Path Planner2D::extractPath(std::vector<Eigen::Vector3d>& pts)
{
    nav_msgs::msg::Path plannedPath;
    plannedPath.header.frame_id = "/map";
    // get the obtained path

    for (unsigned int i = 0; i < pts.size(); ++i) {
        // get state
        geometry_msgs::msg::PoseStamped poseMsg;
        poseMsg.pose.position.x = 0.0;
        poseMsg.pose.position.y = 0.0;
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
    TopologyPRM prm;
    // 1. init
    prm.init(globalMap.info.resolution);

    // 2. find path in each homotopy
    Eigen::Vector3d                 start(0.0, 0.0, 0.0);
    Eigen::Vector3d                 end(5.0, 5.0, 0.0);
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector3d>> raw_paths, filtered_paths, select_paths;

    prm.findTopoPaths(globalMap, start, end, graph, raw_paths, filtered_paths, select_paths);

    // 2.2 plot the graph and filtered_paths

    // 3. smooth and select the optimal path by k-order Markov optimization
    // using thread to caculate the smooth path parallel

    // 8. solve
    bool                         solved = false;
    nav_msgs::msg::Path          plannedPath;
    std::vector<Eigen::Vector3d> pts;
    if (solved) {   // if cussess
        // get the planned path
        plannedPath = extractPath(pts);
    }
    std::cout << "path planned\n";
    return plannedPath;
}

}   // namespace hybrid_planner
