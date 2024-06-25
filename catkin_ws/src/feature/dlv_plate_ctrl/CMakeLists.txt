cmake_minimum_required(VERSION 3.0.2)
project(dlv_plate_ctrl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motor_tool_library src/motor_function.c include/motor_function.h)

add_executable(motor_tool src/motor_tool.cpp)
target_link_libraries(motor_tool motor_tool_library ${catkin_LIBRARIES})

add_executable(motor_comm src/motor_comm.cpp)
target_link_libraries(motor_comm motor_tool_library ${catkin_LIBRARIES})

add_executable(test_sub src/test_sub.cpp)
add_dependencies(test_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sub ${catkin_LIBRARIES})