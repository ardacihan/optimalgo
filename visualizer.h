#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <memory>
#include <GL/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "InstanceGenerator.h"
#include "RectanglePlacement.h"
#include "Solver.h"
// Remove SolverVisualizer for now since it's causing issues
// #include "SolverVisualizer.h"
// #include "RectangleFittingProblem.h"

class RectangleVisualizer {
public:
    RectangleVisualizer(int width, int height);
    ~RectangleVisualizer();


    bool initialize();
    void pollEvents();
    void render();
    bool shouldClose() const;

    void setPlacements(const std::vector<RectanglePlacement>& placements);
    void setBoxLength(int length);

private:
    void generateInstance();
    void runSolver();  // Keep this for future use
    void updateScaleAndOffset();
    void updateMaxSizeLimits();

    void* window;
    int box_length;
    float scale_factor;
    struct { float x, y; } offset;
    bool initialized;



    // Comment out solver_visualizer for now
    // std::unique_ptr<SolverVisualizer> solver_visualizer;
    // std::unique_ptr<RectangleFittingProblem> problem;

    // GUI config
    struct {
        int rect_count = 10;
        int box_size = 100;
        int min_width = 5;
        int max_width = 30;
        int min_height = 5;
        int max_height = 30;
        int current_box_view = 0;
        bool view_all_boxes = true;

        // Solver parameters (keep for future)
        int max_solver_steps = 50;
        int current_solver_step = 0;
        bool show_solver_steps = false;
    } gui_config;



    InstanceGenerator instance_generator =  InstanceGenerator(
        gui_config.box_size,
        gui_config.min_width,
        gui_config.max_width,
        gui_config.min_height,
        gui_config.max_height);
    std::vector<RectanglePlacement> current_placements = instance_generator.generate_rectangles(gui_config.rect_count);

    RectangleFittingProblem problem = RectangleFittingProblem(gui_config.rect_count,current_placements);
    GeometryBasedNeighborhoodSolver solver;
};

#endif