cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(sbpl_dynamic_planner
                     src/envInterval.cpp
                     #                     src/sipp.cpp
                     #src/envDBubble.cpp
                     #src/envTime.cpp
                     src/weightedAStar.cpp
                     src/intervalPlanner.cpp)

rosbuild_add_executable(test_sbpl3D src/test/main.cpp)
target_link_libraries(test_sbpl3D sbpl_dynamic_planner)

rosbuild_add_executable(run_tests src/test/run_tests.cpp)
target_link_libraries(run_tests sbpl_dynamic_planner)

rosbuild_add_executable(pathVisualizer src/pathVisualizer.cpp)

#rosbuild_add_executable(experiments src/test/experiments.cpp)
#target_link_libraries(experiments sbpl_dynamic_planner)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
