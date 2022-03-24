cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(region_grow)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_grow pcl_region_grow.cpp)
target_link_libraries (region_grow ${PCL_LIBRARIES} pcl_visualization)
