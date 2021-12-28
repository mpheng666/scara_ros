cmake_minimum_required(VERSION 3.0.2)
project(scara_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  moveit_ros_planning_interface 
  moveit_ros_move_group
  moveit_visual_tools
  rviz_visual_tools
)

add_message_files(
  FILES
  scara.msg
  TrajectoryJoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  LIBRARIES 
  CATKIN_DEPENDS roscpp 
  DEPENDS
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES src/scara_teleop_node.cpp src/scara_joy.cpp)
set(SOURCES2 src/scara_main.cpp src/scara_planner.cpp)

add_executable(scara_teleop_node ${SOURCES})
add_executable(scara_planner_node ${SOURCES2})

target_link_libraries(scara_teleop_node
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
)
target_link_libraries(scara_planner_node
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_dependencies(scara_planner_node ${catkin_EXPORTED_TARGETS})
install(TARGETS scara_planner_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
