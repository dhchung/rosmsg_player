cmake_minimum_required(VERSION 3.5)

project(rosmsg_player LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    geometry_msgs
    tf
    nav_msgs
    pcl_ros
    pcl_conversions
    pcl_msgs
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        tf
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
                      ${catkin_LIBRARIES})
