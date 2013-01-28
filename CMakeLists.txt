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

SET(CMAKE_VERBOSE_MAKEFILE ON)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
include_directories(src/3rdparty/x52control/libx52)
include_directories(src/3rdparty/hidapi/hidapi)

link_directories(${PROJECT_SOURCE_DIR}/src/3rdparty/lib)

#IF(APPLE)
   FIND_LIBRARY(IOKIT_LIBRARY IOKit)
   FIND_LIBRARY(COREFOUNDATION_LIBRARY CoreFoundation)
   MARK_AS_ADVANCED (IOKIT_LIBRARY
                     COREFOUNDATION_LIBRARY)
   SET(EXTRA_LIBS ${COREFOUNDATION_LIBRARY} ${IOKIT_LIBRARY})

#ENDIF (APPLE)


#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(X52 src/X52.cpp)
target_link_libraries(X52 x52Control)
target_link_libraries(X52 hidapi)
target_link_libraries(X52 ${EXTRA_LIBS})
#target_link_libraries(example ${PROJECT_NAME})
