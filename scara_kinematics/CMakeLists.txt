cmake_minimum_required(VERSION 3.0.2)
project(scara_kinematics)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    # std_srvs
    # geometry_msgs
)

# add_service_files(
#     FILES
#     ScaraIk.srv
#     ScaraFk.srv
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
#     geometry_msgs
# )


catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(scara_kinematics
  src/scara_kinematics.cpp
)
target_link_libraries(scara_kinematics
    ${catkin_LIBRARIES}
    )
add_dependencies(scara_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(scara_kinematics_node
  src/scara_kinematics_node.cpp
)
add_dependencies(scara_kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scara_kinematics_node scara_kinematics)
set_target_properties(scara_kinematics_node PROPERTIES OUTPUT_NAME scara_kinematics)

install(
    TARGETS
        scara_kinematics_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        scara_kinematics
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)