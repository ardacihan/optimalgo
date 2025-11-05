#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "RectanglePlacement.h"
#include "InstanceGenerator.h"
#include "RectangleFittingProblem.h"
#include "Solver.h"

struct GUIConfig {
    int rect_count = 10;
    int box_size = 15;
    int min_width = 5;
    int max_width = 8;
    int min_height = 5;
    int max_height = 8;
    bool view_all_boxes = true;
    int current_box_view = 0;
    bool show_solver_steps = false;
    int max_solver_steps = 2000;
    int num_reruns = 15;
    int max_rectangle_in_subproblem = 50;
};

class RectangleVisualizer {
private:
    void* window;
    int box_length;
    float scale_factor;
    ImVec2 offset;
    bool initialized;
    bool is_solving;

    GUIConfig gui_config;
    InstanceGenerator instance_generator;
    std::vector<RectanglePlacement> current_placements;
    RectangleFittingProblem problem;
    GeometryBasedNeighborhoodSolver solver;

    void updateScaleAndOffset();
    void updateMaxSizeLimits();

public:
    RectangleVisualizer(int width, int height);
    ~RectangleVisualizer();

    bool initialize();
    void setPlacements(const std::vector<RectanglePlacement>& placements);
    void setBoxLength(int length);
    void generateInstance();
    void generateRandomProblem();
    void runSolver();
    void solveNextStep();
    void pollEvents();
    void render();
    bool shouldClose() const;
};

#endif // VISUALIZER_H