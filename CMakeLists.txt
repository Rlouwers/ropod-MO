cmake_minimum_required(VERSION 2.8.3)

project(robot_mo_spawn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
  tf
  rospy
  dynamixel_msgs
  )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simulation
  CATKIN_DEPENDS
    nav_msgs
    roscpp
    rospy
    tf
    std_msgs
    dynamixel_msgs
#  DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)




find_package(roslaunch)

add_executable(move_mo src/move_mo.cpp)
target_link_libraries(move_mo ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
