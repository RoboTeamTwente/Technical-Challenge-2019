cmake_minimum_required(VERSION 3.12)
project(Technical-Challenge-2019)

set(CMAKE_CXX_STANDARD 14)


add_executable(Technical-Challenge-2019
        src/CircularBuffer.cpp
        src/Constants.cpp
        src/Settings.cpp

        src/BallFinder.cpp

        src/Camera.cpp




        src/ImageProcessor.cpp

        src/Interface.cpp


        src/Main.cpp)

find_package(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
TARGET_LINK_LIBRARIES (Technical-Challenge-2019 ${OpenCV_LIBS})