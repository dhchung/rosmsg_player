cmake_minimum_required(VERSION 3.5)

project(rosmsg_player LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
    tf
    tf2
    tf2_ros
    nav_msgs
    pcl_ros
    pcl_conversions
    pcl_msgs
)
find_package(jsoncpp REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        tf
        tf2
        tf2_ros
        nav_msgs
        pcl_ros
        pcl_conversions
        pcl_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rosmsg_player
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    classes/ros_class.h
    classes/ros_class.cpp
)

target_link_libraries(rosmsg_player PRIVATE Qt5::Widgets
                      ${catkin_LIBRARIES}
                      jsoncpp)
