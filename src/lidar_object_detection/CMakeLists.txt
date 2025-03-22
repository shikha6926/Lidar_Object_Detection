cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0144 NEW)  # Fix for FLANN_ROOT warning
cmake_policy(SET CMP0074 NEW)  # Fix for PCL_ROOT warning

project(lidar_object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Executable for publishing PCD
add_executable(publish_pcd src/publish_pcd.cpp)
ament_target_dependencies(publish_pcd rclcpp sensor_msgs pcl_conversions PCL)
target_link_libraries(publish_pcd ${PCL_LIBRARIES})

# Executable for Lidar Node
add_executable(lidar_node src/lidar_node.cpp)
ament_target_dependencies(lidar_node rclcpp sensor_msgs pcl_conversions PCL visualization_msgs)
target_include_directories(lidar_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(lidar_node ${PCL_LIBRARIES})

install(
  DIRECTORY include
  DESTINATION include
)

# Install targets
install(
  TARGETS publish_pcd lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export ament package
ament_package()
