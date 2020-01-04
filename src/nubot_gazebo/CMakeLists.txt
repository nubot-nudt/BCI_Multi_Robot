cmake_minimum_required(VERSION 2.8.3)
project(nubot_gazebo)
set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
cmake_policy(SET CMP0054 NEW)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
  nubot_common
  dynamic_reconfigure
)

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

#generate_dynamic_reconfigure_options(config/nubot_gazebo.cfg)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
    roscpp
    dynamic_reconfigure
)				

#robot_gazebo
add_library(nubot_gazebo src/nubot_gazebo.cpp)
target_link_libraries(nubot_gazebo ${catkin_LIBRARIES}
                                   ${GAZEBO_LIBRARIES}
                                   ${Boost_LIBRARIES}
                                   ${PROTOBUF_LIBRARIES}
                                   pthread)
#add_dependencies(nubot_gazebo ${PROJECT_NAME}_gencfg)
add_dependencies(nubot_gazebo  ${catkin_EXPORTED_TARGETS})

#rival_gazebo
add_library(rival_gazebo src/rival_gazebo.cpp)
target_link_libraries(rival_gazebo ${catkin_LIBRARIES}
                                   ${GAZEBO_LIBRARIES}
                                   ${Boost_LIBRARIES}
                                   ${PROTOBUF_LIBRARIES}
                                   pthread)
#add_dependencies(rival_gazebo ${PROJECT_NAME}_gencfg)
add_dependencies(rival_gazebo  ${catkin_EXPORTED_TARGETS})

#ball_gazebo
add_library(ball_gazebo src/ball_gazebo.cpp)
target_link_libraries(ball_gazebo ${catkin_LIBRARIES}
                                  ${GAZEBO_LIBRARIES}
                                  ${Boost_LIBRARIES}
                                  ${PROTOBUF_LIBRARIES}
                                  pthread)
#add_dependencies(ball_gazebo ${PROJECT_NAME}_gencfg)
add_dependencies(ball_gazebo ${catkin_EXPORTED_TARGETS})

add_executable(nubot_teleop_keyboard src/nubot_teleop_keyboard.cpp)
add_dependencies(nubot_teleop_keyboard  ${catkin_EXPORTED_TARGETS})
target_link_libraries(nubot_teleop_keyboard ${catkin_LIBRARIES})
