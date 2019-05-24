cmake_minimum_required(VERSION 2.8.3)
project(technical-challenge-2019)

# Needed for the documentation generator.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

        roscpp
        rospy
        std_msgs
        roboteam_msgs
        roboteam_utils
        )

## System dependencies are found with CMake's conventions

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        roboteam_vision_convert
        CATKIN_DEPENDS
        message_runtime
)

set(QT_USE_QTNETWORK TRUE)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)





add_executable(Technical-Challenge-2019
        src/CircularBuffer.cpp


        src/BallFinder.cpp

        src/Camera.cpp


        src/ImageProcessor.cpp

        src/Interface.cpp

        src/Connection.cpp

        src/Main.cpp


        src/Publisher.cpp src/Publisher.h)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(Technical-Challenge-2019 ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        ${QT_LIBRARIES}
        ${PROTOBUF_LIBRARIES})