cmake_minimum_required(VERSION 3.0.2)
project(aurova_plugins)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS

)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_library(ActorStopObstaclePlugin SHARED src/ActorStopObstaclePlugin.cc)
target_link_libraries(ActorStopObstaclePlugin ${caktin_LIBRARIES} ${GAZEBO_LIBRARIES})
