cmake_minimum_required(VERSION 3.0.2)
project(osk)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wuninitialized -ltbb")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  rosbag
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS eigen_conversions geometry_msgs message_generation message_runtime nav_msgs pcl_conversions rosbag roscpp sensor_msgs std_msgs tf visualization_msgs
)

find_package(PCL REQUIRED)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
)

add_executable(run_osk src/run_osk.cc)
target_link_libraries(run_osk
    TBB::tbb
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    )