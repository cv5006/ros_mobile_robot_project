cmake_minimum_required(VERSION 2.8.3)
project(mr_project)

set( CMAKE_INCLUDE_PATH "/usr/include")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 genmsg
 message_generation
 rviz
 tf
  tf2
  tf2_ros
  tf2_sensor_msgs)
find_package(Eigen3)
find_package( Boost REQUIRED COMPONENTS thread )
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Use ${EIGEN3_...} variables in either case below

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)
## Declare a catkin package
catkin_package(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf2 tf2_ros )

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/learning_tf.cpp
# )
add_compile_options(-std=c++11)
add_executable(mr_project_node src/mr_project_cpp.cpp)
target_link_libraries(mr_project_node ${catkin_LIBRARIES})
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
