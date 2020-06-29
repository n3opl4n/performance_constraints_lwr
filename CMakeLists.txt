cmake_minimum_required(VERSION 2.8.3)
project(performance_constraints_lwr)

#find_package(Armadillo  7.500 REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(VISION_DIR "")

# Build in release mode
# set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  autharl_core
  std_msgs
  lwr_robot
  performance_constraints
  )

catkin_package()

include_directories(INCLUDE_DIRS
  include
  ${catkin_INCLUDE_DIRS})

add_executable(performance_constraints_lwr
  src/main.cpp
  src/controller.cpp)

add_dependencies(performance_constraints_lwr
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(performance_constraints_lwr
  ${catkin_LIBRARIES}
  lwr-robot
  pthread
  performance-constraints
  )

