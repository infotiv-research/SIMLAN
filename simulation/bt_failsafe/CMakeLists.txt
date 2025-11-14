cmake_minimum_required(VERSION 3.8)
project(bt_failsafe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

include_directories(include)

add_library(bt_failsafe_nodes SHARED
  src/stop_robot.cpp
  src/plugin_registration.cpp
)

ament_target_dependencies(bt_failsafe_nodes
  rclcpp
  geometry_msgs
  tf2_ros
  behaviortree_cpp
  ros_gz_interfaces
  pluginlib
)

target_compile_features(bt_failsafe_nodes PUBLIC cxx_std_17)

set_target_properties(bt_failsafe_nodes PROPERTIES
  CXX_VISIBILITY_PRESET default
  VISIBILITY_INLINES_HIDDEN 0
)

pluginlib_export_plugin_description_file(behaviortree_cpp bt_failsafe_plugins.xml)

install(TARGETS bt_failsafe_nodes
  LIBRARY DESTINATION lib
)

install(FILES bt_failsafe_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
