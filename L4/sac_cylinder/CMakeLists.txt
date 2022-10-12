cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(sac_segmentation)

find_package(PCL 1.2 REQUIRED)
find_package(VTK REQUIRED)

include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sac_segmentation pcl_sac_segmentation.cpp)
target_link_libraries (sac_segmentation ${PCL_LIBRARIES} pcl_visualization ${VTK_LIBRARIES})
