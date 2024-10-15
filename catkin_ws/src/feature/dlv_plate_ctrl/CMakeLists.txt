cmake_minimum_required(VERSION 3.0.2)
project(dlv_plate_ctrl)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


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

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

add_library(motor_tool_library src/motor_function.c include/motor_function.h)

add_executable(motor_tool src/motor_tool.cpp)
target_link_libraries(motor_tool motor_tool_library ${catkin_LIBRARIES})

add_executable(motor_respond_pub src/motor_respond_pub.cpp)
target_link_libraries(motor_respond_pub ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS
  src/move_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/s_shape.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/s_shape_motor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(motor_comm src/motor_comm.cpp)
target_link_libraries(motor_comm motor_tool_library ${catkin_LIBRARIES})

add_executable(test_sub src/test_sub.cpp)
add_dependencies(test_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_sub ${catkin_LIBRARIES})

add_executable(four_wheeled_motor_comm src/four_wheeled_motor_comm.cpp)
target_link_libraries(four_wheeled_motor_comm motor_tool_library ${catkin_LIBRARIES})
add_dependencies(four_wheeled_motor_comm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})