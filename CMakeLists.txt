cmake_minimum_required(VERSION 3.12)
project(Technical-Challenge-2019)

set(CMAKE_CXX_STANDARD 14)


add_executable(Technical-Challenge-2019
        src/BallFinder.cpp
        src/BallFinder.h
        src/Camera.cpp
        src/Camera.h
        src/CircularBuffer.cpp
        src/CircularBuffer.h
        src/Constants.h
        src/ImageProcessor.cpp
        src/ImageProcessor.h
        src/Interface.cpp
        src/Interface.h
        src/Settings.h
        src/Main.h
        src/Main.cpp)

find_package(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
TARGET_LINK_LIBRARIES (Technical-Challenge-2019 ${OpenCV_LIBS})