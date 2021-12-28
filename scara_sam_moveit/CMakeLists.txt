cmake_minimum_required(VERSION 3.1.3)
project(scara_sam_moveit)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
  moveit_ros_planning_interface 
  moveit_ros_move_group
  moveit_visual_tools
  rviz_visual_tools
  scara_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(move_group_interface src/move_group_interface.cpp)
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(move_group_interface ${catkin_EXPORTED_TARGETS})
install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})