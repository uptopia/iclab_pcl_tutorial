cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl_filter_code)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_filter_code pcl_filter_code.cpp)
target_link_libraries (pcl_filter_code ${PCL_LIBRARIES} pcl_visualization pcl_filters)
