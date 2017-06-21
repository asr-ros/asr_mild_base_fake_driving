cmake_minimum_required(VERSION 2.8.3)
project(asr_mild_base_fake_driving)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  gazebo_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS
	thread
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs tf nav_msgs gazebo_msgs geometry_msgs
	DEPENDS Boost
)

include_directories( 
	include 
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/mild_fake_odom.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
