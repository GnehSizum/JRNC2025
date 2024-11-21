cmake_minimum_required(VERSION 3.8)
project(demo_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(behavior_tree REQUIRED)

install(
  DIRECTORY launch config map rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
