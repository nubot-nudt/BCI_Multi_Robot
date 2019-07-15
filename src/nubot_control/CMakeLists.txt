cmake_minimum_required(VERSION 2.8.3)
project(nubot_control)

set(CMAKE_BUILD_TYPE "Debug")
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -Wall -Wno-unused-variable -pthread")
else( CMAKE_BUILD_TYPE STREQUAL "Debug" )
    set(CMAKE_BUILD_TYPE "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -pthread -fopenmp")
endif( CMAKE_BUILD_TYPE STREQUAL "Debug" )

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_generation
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  std_srvs
  nubot_common
  message_filters
)

add_message_files(FILES
   DebugInfo.msg
 )
#generate_dynamic_reconfigure_options(cfg/nubot_control.cfg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_generation
                 dynamic_reconfigure
	         roscpp 
	         rospy  
		 std_msgs 
		 std_srvs 
		 DEPENDS
)

set(nubot_allocation_includes
               ${catkin_INCLUDE_DIRS}
               ${PROJECT_SOURCE_DIR}/include
)

include_directories(${nubot_allocation_includes})

add_executable(nubot_control_node    src/behaviour.cpp
                                     src/subtargets.cpp
                                     src/main.cpp
                                     src/robot_control.cpp
                                     src/role_assignment.cpp
)

target_link_libraries(nubot_control_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )


