cmake_minimum_required(VERSION 3.8)
project(hybrid_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ompl REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  /opt/ros/humble/include
  /opt/ros/humble/include/ompl-1.6
  ${EIGEN3_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
)
message(STATUS "eigen include ${Eigen3_INCLUDE_DIRS}")
message(STATUS ${Eigen3_INCLUDE_DIRS})
message(STATUS "fff ${Eigen_INCLUDE_DIRS}")

## Declare a cpp library
file(GLOB ompl_example_2d_sources
    src//ompl_example_2d/ompl_example_2d.cpp
)

file(GLOB ompl_example_2d_headers
    include/ompl_example_2d/*.hpp
    include/*.h
)

file(GLOB hybrid_planner_sources
    src//hybrid_planner/hybrid_planner.cpp
    src//hybrid_planner/topo_prm.cpp
    src//utils/raycast.cpp
)

file(GLOB hybrid_planner_headers
    include/hybrid_planner/*.hpp
    include/hybrid_planner/*.h
    include/*.h
)


add_library(${PROJECT_NAME}_library STATIC ${ompl_example_2d_sources} ${ompl_example_2d_headers}
)

add_library(hybrid_planner_library STATIC ${hybrid_planner_sources} ${hybrid_planner_headers}
)

target_include_directories(${PROJECT_NAME}_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_library PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(hybrid_planner_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hybrid_planner_library PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  ${PROJECT_NAME}_library
  Eigen3
  "rclcpp"
  "ompl"
  "nav_msgs"
  "geometry_msgs"
)

ament_target_dependencies(
  hybrid_planner_library
  Eigen3
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

# target_link_libraries(${PROJECT_NAME}_library
#    ${amen_LIBRARIES}
# )

add_executable(ompl_example_2d_node src/ompl_example_2d/ompl_example_2d_node.cpp)
target_include_directories(ompl_example_2d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ompl_example_2d_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(ompl_example_2d_node ${PROJECT_NAME}_library ${OMPL_LIBRARIES})
ament_target_dependencies(
  ompl_example_2d_node
  "rclcpp"
  "ompl"
  "nav_msgs"
  "geometry_msgs"
)

add_executable(hybrid_planner_node src/hybrid_planner/hybrid_planner_node.cpp)
target_include_directories(hybrid_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hybrid_planner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(hybrid_planner_node hybrid_planner_library)
ament_target_dependencies(
  hybrid_planner_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
)

# Install launch files
install(
  DIRECTORY launch
  DIRECTORY maps
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ompl_example_2d_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS hybrid_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
