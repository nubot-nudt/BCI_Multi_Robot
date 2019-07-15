cmake_minimum_required(VERSION 3.0.0)
project(bci_background)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
SET(CMAKE_BUILD_TYPE "release")

find_package(catkin REQUIRED COMPONENTS
  image_transport
  message_filters
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  nubot_common
  cmake_modules
)

generate_messages(DEPENDENCIES std_msgs)
 
