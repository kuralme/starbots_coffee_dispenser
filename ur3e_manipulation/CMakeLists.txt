cmake_minimum_required(VERSION 3.8)
project(ur3e_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
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

# Generate action service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DeliverCup.action"
)

add_executable(pick_and_place_server src/pick_and_place_server.cpp)
ament_target_dependencies(pick_and_place_server 
	ament_cmake 
	rclcpp 
	rclcpp_action 
	moveit_core 
	moveit_ros_planning 
	moveit_ros_planning_interface 
	moveit_msgs
	visualization_msgs 
	shape_msgs
	starbots_detection_msgs
)
rosidl_target_interfaces(pick_and_place_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
	pick_and_place_server
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
	action
	rviz
    launch
    DESTINATION share/${PROJECT_NAME}
)
ament_package()