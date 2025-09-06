cmake_minimum_required(VERSION 3.8)
project(pointcloud_filter)

# Suppress pedantic warnings for PCL
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
ament_target_dependencies(pointcloud_filter_node rclcpp sensor_msgs PCL pcl_conversions)

install(TARGETS
  pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
