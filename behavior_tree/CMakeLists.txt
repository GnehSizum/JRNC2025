cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(patrol_node 
  src/main.cpp
)

ament_target_dependencies(patrol_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs
)

install(TARGETS patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
