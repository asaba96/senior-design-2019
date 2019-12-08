cmake_minimum_required(VERSION 2.8.3)
project(mbz2020_planner)

# force release and c++ 11
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS)

catkin_metapackage()
