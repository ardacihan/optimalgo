cmake_minimum_required(VERSION 3.15)
project(optimalgo)

set(CMAKE_CXX_STANDARD 20)

# Find packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(glfw REQUIRED glfw3)
find_package(OpenGL REQUIRED)

# ImGui setup
set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/imgui)

# Verify ImGui directory exists
if(NOT EXISTS ${IMGUI_DIR}/imgui.h)
    message(FATAL_ERROR "ImGui not found at ${IMGUI_DIR}")
endif()

set(IMGUI_SOURCES
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_library(imgui STATIC ${IMGUI_SOURCES})

# Use PUBLIC (not INTERFACE) so the imgui library itself can find its headers during compilation
target_include_directories(imgui PUBLIC
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)

target_link_libraries(imgui PUBLIC ${glfw_LIBRARIES} OpenGL::GL)

# Your executable
add_executable(optimalgo
        RectanglePlacement.cpp
        InstanceGenerator.cpp
        OptimizationProblem.cpp
        RectangleFittingProblem.cpp
        GeometryBasedNeighborhoodSolver.cpp
        RuleBasedNeighborhoodSolver.cpp
        visualizer.cpp
        main_vis.cpp
)

# Use PUBLIC to propagate include directories
target_include_directories(optimalgo PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${glfw_INCLUDE_DIRS}
        ${IMGUI_DIR}
        ${IMGUI_DIR}/backends
)

target_link_libraries(optimalgo PUBLIC
        imgui
        ${glfw_LIBRARIES}
        OpenGL::GL
        X11
        pthread
        dl
)