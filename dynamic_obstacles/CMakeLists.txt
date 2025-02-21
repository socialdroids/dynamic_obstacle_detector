cmake_minimum_required(VERSION 3.8)
project(dynamic_obstacles)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(people_msgs REQUIRED)  
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)  

ament_export_dependencies(
  rclcpp
  geometry_msgs
  laser_geometry
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  people_msgs  
  visualization_msgs
  std_msgs
)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/dynamic_obstacle_detector.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  laser_geometry
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  people_msgs 
  visualization_msgs
  std_msgs
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
