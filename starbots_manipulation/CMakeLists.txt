cmake_minimum_required(VERSION 3.8)
project(starbots_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(starbots_detection_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeliverCup.srv"
)

add_executable(starbots_delivery_server src/starbots_delivery_server.cpp)
ament_target_dependencies(starbots_delivery_server 
	ament_cmake 
	rclcpp 
	moveit_core 
	moveit_ros_planning 
	moveit_ros_planning_interface 
	moveit_msgs
	visualization_msgs 
	shape_msgs
	starbots_detection_msgs
)
rosidl_target_interfaces(starbots_delivery_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
	starbots_delivery_server
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
	srv
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()