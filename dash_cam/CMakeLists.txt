cmake_minimum_required(VERSION 2.8.3)
project(dash_cam)
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-lwiringPi -I/usr/local/include -L/usr/local/lib")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nodelet
)

find_package(OpenCV REQUIRED core imgproc highgui videoio)

FIND_LIBRARY(WIRINGPI_LIBRARY wiringPi /home/ubuntu/wiringPi)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
	/usr/local/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/dash_cam_node.cpp src/dash_cam_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${WIRINGPI_LIBRARY} ${OpenCV_LIB} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


