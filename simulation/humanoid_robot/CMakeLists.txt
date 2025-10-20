cmake_minimum_required(VERSION 3.8)
project(humanoid_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch worlds config rviz model
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files in the package/share directory
file(GLOB HUMAN_WITH_MESHES_URDF model/human-gazebo/humanSubjectWithMeshes/*.urdf)

install(
  FILES ${HUMAN_WITH_MESHES_URDF}
  DESTINATION share/${PROJECT_NAME})

# Install mesh files in the package/share/meshes directory
file(GLOB HUMAN_WITH_MESHES model/human-gazebo/humanSubjectWithMeshes/meshes/*.stl)

install(
  FILES ${HUMAN_WITH_MESHES}
  DESTINATION share/${PROJECT_NAME}/meshes)


ament_package()
