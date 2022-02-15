cmake_minimum_required(VERSION 3.0.2)
project(pi4_peristaltic_pump_driver)


find_package(catkin REQUIRED COMPONENTS
  rospy
  pi4_peristaltic_pump_msgs
)


catkin_python_setup()


catkin_package(
	CATKIN_DEPENDS pi4_peristaltic_pump_msgs)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/perstaltic_pump_driver
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
