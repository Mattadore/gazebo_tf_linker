cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tf_linker)

set(MSG_DEPS
    std_msgs
    geometry_msgs
    gazebo_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
  tf2
  rospy
)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  DEPENDS
  CATKIN_DEPENDS
        std_msgs
        roscpp
        tf
        tf2
        gazebo_ros
        geometry_msgs
        gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

set(GENCPP_DEPS gazebo_msgs_gencpp std_msgs_gencpp geometry_msgs_gencpp)

add_executable(linker nodes/cpplinker.cpp)
target_link_libraries(linker ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(linker ${PROJECT_NAME}_gencpp ${GENCPP_DEPS})

#############
## Install ##
#############

install(TARGETS ${TFLINK_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS
  scripts/linker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
