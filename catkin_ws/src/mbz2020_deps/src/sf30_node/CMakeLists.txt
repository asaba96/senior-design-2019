cmake_minimum_required(VERSION 2.8.3)
project(sf30_node)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES s30_node
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/sf30_node.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


