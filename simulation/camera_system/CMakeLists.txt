cmake_minimum_required(VERSION 3.8)
project(camera_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(xacro REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureMotionStatus.srv"
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  src/camera_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
