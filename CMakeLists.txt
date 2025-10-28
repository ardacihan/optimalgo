cmake_minimum_required(VERSION 4.0)
project(untitled)

set(CMAKE_CXX_STANDARD 20)

add_executable(untitled main.cpp
        RectanglePlacement.cpp
        RectanglePlacement.h
        InstanceGenerator.cpp
        InstanceGenerator.h
        OptimizationProblem.cpp
        OptimizationProblem.h
        RectangleFittingProblem.cpp
        RectangleFittingProblem.h
        Solver.cpp
        Solver.h
        Input.h
)
