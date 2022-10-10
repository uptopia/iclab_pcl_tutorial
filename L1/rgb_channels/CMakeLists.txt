cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(rgb_channels)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rgb_channels rgb_channels.cpp)
target_link_libraries (rgb_channels ${PCL_LIBRARIES} pcl_visualization)
