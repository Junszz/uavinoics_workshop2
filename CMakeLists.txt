cmake_minimum_required(VERSION 2.8.3)

project(uavcar)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(roslaunch)

find_package(catkin REQUIRED COMPONENTS
	gazebo_ros
	roscpp
	geometry_msgs 
	sensor_msgs
	nav_msgs
	std_srvs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
	CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs std_srvs
    INCLUDE_DIRS
    LIBRARIES
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
