cmake_minimum_required(VERSION 3.8)
project(barista_dock_charger_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DockChargerStation.srv"
  "msg/FleetDockStatus.msg"
)


install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/start_dock_charger_station_server.py
  scripts/start_fleet_robots_are_docked.py
  scripts/barista_dock_node.py
  scripts/barista_dock_node_v2.py
  scripts/barista_dock_node_v3.py
  scripts/scan_orientation.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
