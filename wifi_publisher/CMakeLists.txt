cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project(wifi_publisher)

#rosbuild_init()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
        genmsg
        roscpp
        rospy
        std_msgs
        message_generation
        cv_bridge
        image_transport
        tf
)

find_package(OpenCV REQUIRED)

add_message_files(
        FILES
        wifi.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES occgrid_planner
        CATKIN_DEPENDS message_runtime cv_bridge roscpp rospy tf
        #  DEPENDS system_lib
)

include_directories(
        ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



## Declare a cpp executable
add_executable(wifi_signal_grid_builder ./src/WifiSignalGridBuilder.cpp ./src/WifiSignalGridBuilder.h)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(wifi_signal_grid_builder wifi_publisher_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(wifi_signal_grid_builder ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

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
