cmake_minimum_required(VERSION 3.5)
project(frontier_exploration)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(frontier_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

#Add libraries
add_library(frontier_exploration_core SHARED plugins/bounded_explore_layer.cpp src/frontier_search.cpp)
ament_target_dependencies(frontier_exploration_core rclcpp pluginlib geometry_msgs nav2_costmap_2d sensor_msgs frontier_msgs pcl_ros pcl_conversions PCL)
target_link_libraries(frontier_exploration_core ${PCL_LIBRARIES})


# Add executable 1
add_executable(explore_client src/explore_client.cpp)
ament_target_dependencies(explore_client rclcpp geometry_msgs std_msgs nav2_msgs nav2_costmap_2d visualization_msgs frontier_msgs rclcpp_action)

# Add executable 2
add_executable(explore_server src/explore_server.cpp)
ament_target_dependencies(explore_server rclcpp nav2_msgs nav2_costmap_2d visualization_msgs frontier_msgs rclcpp_action action_msgs)

# # Add executable 3 (Temporary)
# add_executable(bounded_explore_layer src/frontier_search.cpp)
# ament_target_dependencies(bounded_explore_layer rclcpp pluginlib geometry_msgs nav2_costmap_2d sensor_msgs frontier_msgs pcl_ros pcl_conversions PCL)

# Add executable 4
add_executable(polygon_point_publisher src/polygon_point_publisher.cpp)
ament_target_dependencies(polygon_point_publisher rclcpp geometry_msgs)

# TODO
# bounded_explore_layer
install(TARGETS explore_server explore_client frontier_exploration_core polygon_point_publisher
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_costmap_2d bounded_explore_layer.xml)
ament_export_dependencies(rosidl_default_runtime)

ament_package()