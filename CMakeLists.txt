cmake_minimum_required(VERSION 3.15)
project(optimalgo)

set(CMAKE_CXX_STANDARD 20)

# Find packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(glfw REQUIRED glfw3)
find_package(OpenGL REQUIRED)

# Download ImGui automatically
include(FetchContent)
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.90.8
)
FetchContent_MakeAvailable(imgui)

# ImGui setup
set(IMGUI_SOURCES
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        src/solver/RectangleFittingProblemSolver.cpp
        src/solver/RectangleFittingProblemSolver.h
)

add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(imgui PUBLIC ${glfw_LIBRARIES} OpenGL::GL)

# Your executable
add_executable(optimalgo
        src/problem/primitives/RectanglePlacement.cpp
        src/problem/primitives/InstanceGenerator.cpp
        src/problem/RectangleFittingProblem.cpp
        src/solver/local_search/specialized_solvers/GeometryBasedNeighborhoodSolver.cpp
        src/solver/local_search/specialized_solvers/RuleBasedNeighborhoodSolver.cpp
        src/gui/visualizer.cpp
        src/gui/main_vis.cpp
        src/solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.cpp
        src/solver/greedy/GreedySolver.cpp
        src/Benchmark.cpp
        src/Benchmark.h
        src/gui/benchmark_loader.cpp
        src/gui/benchmark_loader.h
)

target_include_directories(optimalgo PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${glfw_INCLUDE_DIRS}
)

target_link_libraries(optimalgo PUBLIC
        imgui
        ${glfw_LIBRARIES}
        OpenGL::GL
        X11
        pthread
        dl
)