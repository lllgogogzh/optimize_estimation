cmake_minimum_required(VERSION 3.0.2)
project(optimize_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)


add_executable(imu_test_node 
  src/imu_pose_test.cpp
)

target_link_libraries(imu_test_node
  ${catkin_LIBRARIES}
)

add_executable(imu_tf_pose 
  src/imu_tf_pose.cpp
)

target_link_libraries(imu_tf_pose 
  ${catkin_LIBRARIES}
)# ${SERIAL_LIB}

add_executable(dyna_pose_test
  src/dyna_pose_test.cpp
)

target_link_libraries(dyna_pose_test 
  ${catkin_LIBRARIES}
)