#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "RectanglePlacement.h"
#include "InstanceGenerator.h"
#include "RectangleFittingProblem.h"
#include "GeometryBasedNeighborhoodSolver.h"
#include "RuleBasedNeighborhoodSolver.h"

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
    int num_reruns = 2;
    int max_rectangle_in_subproblem = 50;

    // Strategy selection
    int neighborhood_strategy = 0; // 0 = Geometry Based, 1 = Permutation Based
};

class RectangleVisualizer {
private:
    void* window;
    int box_length;
    float scale_factor;
    ImVec2 offset;
    bool initialized;

    GUIConfig gui_config;
    InstanceGenerator instance_generator;
    std::vector<RectanglePlacement> current_placements;
    std::vector<RectanglePlacement> original_placements;
    RectangleFittingProblem problem;
    std::unique_ptr<GeometryBasedNeighborhoodSolver> geometry_solver;
    std::unique_ptr<RuleBasedNeighborhoodSolver> permutation_solver;

    // Threading support for solver
    std::thread solver_thread;
    std::atomic<bool> is_solving;
    std::atomic<bool> solver_thread_active;
    std::mutex solver_mutex;
    std::vector<RectanglePlacement> pending_result;
    std::chrono::steady_clock::time_point solver_start_time;

    void updateScaleAndOffset();
    void updateMaxSizeLimits();
    void saveOriginalState();

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
    void revertToOriginal();
    void pollEvents();
    void render();
    bool shouldClose() const;
};

#endif // VISUALIZER_H