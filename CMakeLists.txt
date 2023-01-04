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
  mavros
)
find_package(PythonLibs)

catkin_package(

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PYTHON_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

add_library(
  oplib
  src/imu_ekf.cpp
)

add_executable(imu_test_node 
  src/imu_pose_test.cpp
)

target_link_libraries(imu_test_node
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  oplib
)

target_include_directories(imu_test_node
    PRIVATE 
    ${PYTHON_INCLUDE_DIRS}
)

add_executable(imu_tf_pose 
  src/imu_tf_pose.cpp
)

target_link_libraries(imu_tf_pose 
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  oplib
)# ${SERIAL_LIB}

target_include_directories(imu_tf_pose 
    PRIVATE 
    ${PYTHON_INCLUDE_DIRS}
)

add_executable(dyna_pose_test
  src/dyna_pose_test.cpp
)

target_link_libraries(dyna_pose_test
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  oplib
)

target_include_directories(dyna_pose_test
    PRIVATE 
    ${PYTHON_INCLUDE_DIRS}
)

add_executable(imu_ekf_node
  src/imu_ekf_node.cpp
)

target_link_libraries(imu_ekf_node
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  oplib
)

target_include_directories(imu_ekf_node
    PRIVATE 
    ${PYTHON_INCLUDE_DIRS}
)

