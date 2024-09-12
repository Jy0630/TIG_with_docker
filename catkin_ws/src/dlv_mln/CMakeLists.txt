cmake_minimum_required(VERSION 2.8.3)

project(dlv_mln)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  tf      
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mecanum_odom_for_fusion src/mecanum_odom_for_fusion.cpp)
target_link_libraries(mecanum_odom_for_fusion ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/odom_to_base_tf_broadcaster.py
  src/odom_to_base_tf_broadcaster_for_fusion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch rviz config maps)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)