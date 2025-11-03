cmake_minimum_required(VERSION 3.15)
project(optimalgo)

set(CMAKE_CXX_STANDARD 20)

# Find system GLFW
find_package(PkgConfig REQUIRED)
pkg_check_modules(glfw REQUIRED glfw3)

# Download ImGui
include(FetchContent)

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.90.8
)

FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)

    # ImGui sources
    set(IMGUI_SOURCES
            ${imgui_SOURCE_DIR}/imgui.cpp
            ${imgui_SOURCE_DIR}/imgui_draw.cpp
            ${imgui_SOURCE_DIR}/imgui_tables.cpp
            ${imgui_SOURCE_DIR}/imgui_widgets.cpp
            ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
            ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )

    add_library(imgui STATIC ${IMGUI_SOURCES})
    target_include_directories(imgui PUBLIC
            ${imgui_SOURCE_DIR}
            ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC ${glfw_LIBRARIES})
endif()

# Your main executable
add_executable(optimalgo
        main.cpp
        RectanglePlacement.cpp
        InstanceGenerator.cpp
        OptimizationProblem.cpp
        RectangleFittingProblem.cpp
        Solver.cpp

        # Visualization files
        visualizer.cpp
        main_vis.cpp
)

# Include directories
target_include_directories(optimalgo PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${glfw_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(optimalgo
        ${glfw_LIBRARIES}
        imgui
        GL
)

# Platform-specific libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(optimalgo
            X11
            pthread
            dl
    )
endif()