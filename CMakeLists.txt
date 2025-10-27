cmake_minimum_required(VERSION 4.0)
project(untitled)

set(CMAKE_CXX_STANDARD 20)

add_executable(untitled main.cpp
        Rectangle.cpp
        Rectangle.h
        RectanglePlacement.cpp
        RectanglePlacement.h
        BoundingBoxPlacement.cpp
        BoundingBoxPlacement.h
        InstanceGenerator.cpp
        InstanceGenerator.h
)
