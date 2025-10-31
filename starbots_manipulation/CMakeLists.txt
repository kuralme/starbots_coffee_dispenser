cmake_minimum_required(VERSION 3.8)
project(starbots_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(starbots_detection_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Interface generation ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DeliverCup.srv"
  DEPENDENCIES builtin_interfaces
)

# === Executable ===
add_executable(starbots_delivery_server
  src/bt_main.cpp
  src/pick_and_place.cpp
  src/bt_nodes/validate_detection_node.cpp
  src/bt_nodes/prepick_node.cpp
  src/bt_nodes/pick_node.cpp
  src/bt_nodes/preplace_node.cpp
  src/bt_nodes/place_node.cpp
  src/bt_nodes/putback_node.cpp
  src/bt_nodes/return_node.cpp
  src/observer.cpp
)
target_include_directories(starbots_delivery_server PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)
ament_target_dependencies(starbots_delivery_server 
  rclcpp 
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_msgs
  visualization_msgs 
  shape_msgs
  starbots_detection_msgs
  behaviortree_cpp
)

# === Link service typesupport ===
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(starbots_delivery_server ${cpp_typesupport_target})

# === Install ===
install(TARGETS 
  starbots_delivery_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY 
  bt_config
  srv
  launch
  DESTINATION share/${PROJECT_NAME}
)

# === Export runtime deps ===
ament_export_dependencies(rosidl_default_runtime)

ament_package()
