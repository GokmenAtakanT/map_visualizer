cmake_minimum_required(VERSION 3.5)
project(map_visualizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_maps REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_validation REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dep_osm "std_msgs" "nav_msgs" "geometry_msgs"  "rclcpp" "visualization_msgs" "lanelet2_core" "lanelet2_io" "lanelet2_maps" "lanelet2_validation" "lanelet2_routing" "lanelet2_projection" "lanelet2_traffic_rules")
set(dep_ocm "rclcpp" "Eigen3" "std_msgs" "nav_msgs" "tf2" "geometry_msgs" )

include_directories(include())

add_executable(osm_visualizer_node
        src/osmVisualizer/osmVisualizer2.cpp
        src/osmVisualizer/osm_visualizer_node.cpp)

ament_target_dependencies(osm_visualizer_node ${dep_osm})

target_include_directories(osm_visualizer_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS osm_visualizer_node
        DESTINATION lib/${PROJECT_NAME})



### occupancy grid


include_directories(include())

add_executable(occupancyGrid_node
        src/occupancyGrid/occupancyGrid_node.cpp
        src/occupancyGrid/occupancyGrid.cpp
)
ament_target_dependencies(occupancyGrid_node ${dep_ocm})

target_include_directories(occupancyGrid_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS occupancyGrid_node
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()