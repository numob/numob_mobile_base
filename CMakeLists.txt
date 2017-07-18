cmake_minimum_required(VERSION 2.8.3)
project(numob_mobile_base)

set(CMAKE_CXX_STANDARD 11)

## Catkin packages on ROS host.
if(UNIX AND NOT APPLE)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf  geometry_msgs sensor_msgs)
catkin_package()
endif()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(numob_mobile_base src/mobile_base.cpp)
target_link_libraries(numob_mobile_base ${catkin_LIBRARIES}  numob_sdk)

add_executable(laser_footprint_filter src/laser_footprint_filter.cpp)
target_link_libraries(laser_footprint_filter ${catkin_LIBRARIES} )


#For remote development on MacOS X. Build this package using catkin_make on ROS host, Not on Mac OS X.
if(APPLE)
    #/net/pro.local is mounted network disk
    include_directories(/net/pro.local/usr/include)
    include_directories(/net/pro.local/usr/local/include)
    include_directories(/net/pro.local/opt/ros/kinetic/include)
endif()