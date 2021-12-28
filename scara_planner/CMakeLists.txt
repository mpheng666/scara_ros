cmake_minimum_required(VERSION 3.0.2)
project(scara_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  scara.msg
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

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES src/scara_teleop_node.cpp src/scara_joy.cpp)
# set(SOURCES2 src/scara_main.cpp src/scara_planner.cpp)

add_executable(scara_teleop_node ${SOURCES})
# add_executable(scara_node ${SOURCES2})

target_link_libraries(scara_teleop_node
   ${roscpp_LIBRARIES}
   ${catkin_LIBRARIES}
)
# target_link_libraries(scara_node
#    ${roscpp_LIBRARIES}
#    ${catkin_LIBRARIES}
# )
