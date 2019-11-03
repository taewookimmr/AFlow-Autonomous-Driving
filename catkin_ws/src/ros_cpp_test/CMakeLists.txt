cmake_minimum_required(VERSION 2.8.3)
project(ros_cpp_test)

## set(CMAKE_BUILD_TYPE Release) ## when making shared library file

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    darknet_ros_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

find_package(OpenCV REQUIRED)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES
    std_msgs
    darknet_ros_msgs
)

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   sensor_msgs
                   std_msgs
                   cv_bridge
                   image_transport
)

## Build talker and listener
include_directories(
    # include
    src
    home/nvidia/catkin_ws/devel/include
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(listener src/listener.cpp src/JHPWMPCA9685.cpp src/OpenCV.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
## add_dependencies(listener ros_cpp_test_generate_messages_cpp)


add_executable(image_node src/image_node.cpp src/OpenCV.cpp)
target_link_libraries(image_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


## add_library(twk SHARED src/listener.cpp src/JHPWMPCA9685.cpp src/OpenCV.cpp) ## when making shared library file

