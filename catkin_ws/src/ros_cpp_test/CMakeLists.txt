cmake_minimum_required(VERSION 2.8.3)
project(ros_cpp_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    darknet_ros_msgs
)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES
    std_msgs
    darknet_ros_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
    # include
    src
    home/nvidia/catkin_ws/devel/include
    ${catkin_INCLUDE_DIRS} 
)

add_executable(listener src/listener.cpp src/JHPWMPCA9685.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ros_cpp_test_generate_messages_cpp)

