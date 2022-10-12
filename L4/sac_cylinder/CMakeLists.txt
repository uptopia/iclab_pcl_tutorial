cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(sac_cylinder)

find_package(PCL 1.2 REQUIRED)
set(VTK_DIR "/usr/include;/usr/include/vtk-7.1")
find_package(VTK REQUIRED)

message(STATUS "PCL status:")
message(STATUS "    version: ${PCL_VERSION}")
message(STATUS "    library_dirs: ${PCL_LIBRARY_DIRS}")
message(STATUS "    PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
#message(STATUS "    libraries: ${PCL_LIBRARIES}")

message(STATUS "VTK status:")
message(STATUS "    version: ${VTK_VERSION}")
message(STATUS "    library_dirs: ${VTK_DIR}")
message(STATUS "    VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")
#message(STATUS "    libraries: ${VTK_LIBRARIES}")

include(${VTK_USE_FILE})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (sac_cylinder sac_cylinder.cpp)
target_link_libraries (sac_cylinder ${PCL_LIBRARIES} pcl_visualization ${VTK_LIBRARIES})
