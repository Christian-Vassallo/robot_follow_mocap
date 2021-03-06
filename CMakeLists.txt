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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(src/API_Turtlebot.hpp )
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# Path Following
set(SRCS1
    ./src/robot_control.cpp)

# Kalmen and Butt filter for walking velocity estimatio
set(SRCS2
    ./src/velocity_estimation.cpp)

# Tracking control
set(SRCS3
    ./src/tracking_control.cpp)

set(SRCS4
    ./src/manip_auto.cpp)

set(HDRS
  ./src/turtlemotionclass.hpp
  ./src/mocapmessenger.hpp
  ./src/trajectoryClass.hpp
  ./src/Robulab10Class.hpp
  ./src/Motion_API.hpp
  ./src/tracking_API.hpp
   )

rosbuild_add_executable(robotcontrol ${HDRS} ${SRCS1})
rosbuild_add_executable(velocitykalman ${HDRS} ${SRCS2})
rosbuild_add_executable(eventcheck ${HDRS} ${SRCS3})
rosbuild_add_executable(manipRennes ${HDRS} ${SRCS4})



#rosbuild_add_executable(event_check src/new_event_check.cpp)
#rosbuild_add_executable(robot_rotation src/rotaterobot.cpp)
#rosbuild_add_executable(randommotion src/easymotionforward.cpp)
#rosbuild_add_executable(fakemotion src/fakemotion.cpp)
