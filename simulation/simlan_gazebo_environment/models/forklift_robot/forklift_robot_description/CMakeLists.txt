cmake_minimum_required(VERSION 3.10)
project(forklift_robot_description)

find_package(ament_cmake REQUIRED)


install(
    DIRECTORY forklift_webots
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
