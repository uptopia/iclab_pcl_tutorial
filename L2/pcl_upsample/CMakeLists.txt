cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcl_upsample)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_upsample pcl_upsample.cpp)
target_link_libraries (pcl_upsample ${PCL_LIBRARIES} pcl_visualization)
