cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(region_grow_rgb)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (region_grow_rgb pcl_region_grow_rgb.cpp)
target_link_libraries (region_grow_rgb ${PCL_LIBRARIES} pcl_visualization)
