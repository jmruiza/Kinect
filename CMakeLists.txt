cmake_minimum_required(VERSION 2.8)

# Project name (General)
project(Kinect)

# Build Type: Debug / Release / etc
set(CMAKE_BUILD_TYPE "Debug")

# Finds libraries and sets all required variables,

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(source)
