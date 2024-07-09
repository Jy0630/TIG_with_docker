cmake_minimum_required(VERSION 2.8.3)

project(dlv_description)

find_package(catkin REQUIRED)

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  robot_state_publisher
  rviz
)

foreach(dir launch rviz urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)