cmake_minimum_required(VERSION 2.8.3)
project(robot_trailer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    gazebo_msgs
    roscpp
    rospy
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    urdf
    controller_manager
    joint_state_controller
    robot_state_publisher
)

find_package(Boost REQUIRED COMPONENTS thread)

# catkin_python_setup()

catkin_package(
)

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)
