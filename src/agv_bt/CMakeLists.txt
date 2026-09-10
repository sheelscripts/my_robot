cmake_minimum_required(VERSION 3.8)
project(agv_bt CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_library(nav2_pause_resume_controller_bt_node SHARED
  src/pause_resume_controller.cpp
)

target_compile_definitions(nav2_pause_resume_controller_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(nav2_pause_resume_controller_bt_node
  rclcpp
  behaviortree_cpp
  std_srvs
)

install(TARGETS nav2_pause_resume_controller_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES nav2_tree_nodes.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
