cmake_minimum_required(VERSION 3.12)
project(Technical-Challenge-2019)

set(CMAKE_CXX_STANDARD 14)


add_executable(Technical-Challenge-2019 src/Main.cpp src/Settings.h src/CircularBuffer.cpp src/CircularBuffer.h)

find_package(OpenCV REQUIRED )
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )
TARGET_LINK_LIBRARIES (Technical-Challenge-2019 ${OpenCV_LIBS})