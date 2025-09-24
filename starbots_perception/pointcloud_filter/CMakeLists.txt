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

add_executable(pcl_qos_conv src/pcl_qos_conv.cpp)
ament_target_dependencies(pcl_qos_conv rclcpp sensor_msgs PCL)

install(TARGETS
  pointcloud_filter_node
  pcl_qos_conv
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()
