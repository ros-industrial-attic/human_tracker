cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(VTK REQUIRED)
if(VTK_FOUND)
	include_directories(${VTK_INCLUDE_DIRS})
	link_directories(${VTK_LIBRARY_DIRS})
	add_definitions(${VTK_DEFINITIONS})
	include (${VTK_USE_FILE})
endif() 

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(${PROJECT_NAME} src/ground_based_people_detector_node.cpp)
target_link_libraries(${PROJECT_NAME} vtkHybrid vtkRendering)

