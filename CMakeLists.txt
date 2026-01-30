cmake_minimum_required(VERSION 3.8)
project(noid_lifter_mover)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(seed_ros2_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)

set(STROKE_CONVERTER ${PROJECT_NAME}_stroke_converter)
set(HEADER_DIRS ${HEADER_DIRS} ${PROJECT_SOURCE_DIR}/../../libs/rt_logger/src)

add_library(${STROKE_CONVERTER} SHARED stroke_converter/src/stroke_converter.cpp)
target_include_directories(${STROKE_CONVERTER} PRIVATE ${HEADER_DIRS})
target_compile_options(${STROKE_CONVERTER} PUBLIC -O0 -g -Wall)

ament_target_dependencies(
  ${STROKE_CONVERTER} PUBLIC
  pluginlib
  rclcpp
  seed_ros2_controller
)

install(DIRECTORY
  config
  launch
  model
  moveit
  rviz
  stroke_converter/csv
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${STROKE_CONVERTER}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${STROKE_CONVERTER})
ament_export_targets(export_${PROJECT_NAME})

pluginlib_export_plugin_description_file(seed_ros2_controller stroke_converter/plugin_description.xml)

add_executable(tf_static_monitor_node src/tf_static_monitor_node.cpp)
add_executable(tf_static_ready_monitor_node src/tf_static_ready_monitor_node.cpp)
add_executable(save_map_client_node src/save_map_client_node.cpp)
add_executable(bringup_navigation_monitor_node src/bringup_navigation_monitor_node.cpp)
add_executable(amcl_state_monitor_node src/amcl_state_monitor_node.cpp)
add_executable(controllers_state_monitor_node src/controllers_state_monitor_node.cpp)
add_executable(bringup_moveit_monitor_node src/bringup_moveit_monitor_node.cpp)
add_executable(bringup_rviz2_monitor_node src/bringup_rviz2_monitor_node.cpp)

ament_target_dependencies(bringup_rviz2_monitor_node
  rclcpp
  rclcpp_action
  moveit_msgs
)

ament_target_dependencies(tf_static_monitor_node
  rclcpp
  std_msgs
  tf2_msgs
)

ament_target_dependencies(tf_static_ready_monitor_node
  rclcpp
  std_msgs
  ament_index_cpp
)

ament_target_dependencies(save_map_client_node
  rclcpp
  nav2_msgs
)

ament_target_dependencies(bringup_navigation_monitor_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(bringup_moveit_monitor_node
  rclcpp
  lifecycle_msgs
  controller_manager_msgs
  std_msgs
)

ament_target_dependencies(controllers_state_monitor_node
  rclcpp
  lifecycle_msgs
  std_msgs
)

ament_target_dependencies(amcl_state_monitor_node
  rclcpp
  lifecycle_msgs
)

install(TARGETS
  tf_static_monitor_node
  tf_static_ready_monitor_node
  save_map_client_node
  bringup_navigation_monitor_node
  controllers_state_monitor_node
  amcl_state_monitor_node
  bringup_rviz2_monitor_node
  bringup_moveit_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() # プラグインライブラリのリストを作成する
