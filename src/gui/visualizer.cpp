#include "visualizer.h"
#include <algorithm>
#include <unordered_set>
#include <sstream>
#include <iomanip>
#include <GLFW/glfw3.h>
#include "imgui_internal.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "../solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.h"
#include "../Benchmark.h"
#include "benchmark_loader.h"

static int g_changed_rect_idx = -1;
static bool g_objective_improved = false;

void RectangleVisualizer::reset_relaxed_temperature() {
    if (relaxed_geometry_solver) {
        gui_config.T = 1000;
    }
}

void RectangleVisualizer::updateT() {
    double current_temperature = gui_config.T;
    double min_temperature = 0.0;
    double cooling_rate = 0.97;
    current_temperature = current_temperature * std::pow(cooling_rate, 1);
    current_temperature = std::max(min_temperature, current_temperature);
    gui_config.T = (int)current_temperature;
}

RectangleVisualizer::RectangleVisualizer(int width, int height)
    : box_length(15), scale_factor(1.0f), offset{50.0f, 50.0f}, initialized(false),
      is_solving(false), solver_thread_active(false),
      is_benchmarking(false), benchmark_thread_active(false),
      benchmark_status("Idle"),
      instance_generator(
          gui_config.box_size,
          gui_config.min_width,
          gui_config.max_width,
          gui_config.min_height,
          gui_config.max_height
      ),
      problem(gui_config.box_size, std::vector<RectanglePlacement>()) {

    if (!glfwInit()) {
        return;
    }

    generateRandomProblem();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window = glfwCreateWindow(width, height, "Rectangle Packing Visualization", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(static_cast<GLFWwindow *>(window));
    glfwSwapInterval(1);
}

RectangleVisualizer::~RectangleVisualizer() {
    if (solver_thread.joinable()) {
        solver_thread.join();
    }

    if (benchmark_thread.joinable()) {
        benchmark_thread.join();
    }

    if (initialized) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }

    if (window) {
        glfwDestroyWindow(static_cast<GLFWwindow*>(window));
    }
    glfwTerminate();
}

bool RectangleVisualizer::initialize() {
    if (!window) {
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(static_cast<GLFWwindow*>(window), true)) {
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        return false;
    }

    initialized = true;
    return true;
}

void RectangleVisualizer::setPlacements(const std::vector<RectanglePlacement>& placements) {
    current_placements = placements;
    updateScaleAndOffset();
}

void RectangleVisualizer::setBoxLength(int length) {
    box_length = length;
    updateScaleAndOffset();
    updateMaxSizeLimits();
}

void RectangleVisualizer::updateScaleAndOffset() {
    float max_coord = box_length * 1.2f;
    scale_factor = std::min(600.0f / max_coord, 10.0f);
    offset = {100.0f, 150.0f};
}

void RectangleVisualizer::updateMaxSizeLimits() {
    if (gui_config.max_width > box_length) {
        gui_config.max_width = box_length;
    }
    if (gui_config.max_height > box_length) {
        gui_config.max_height = box_length;
    }
    if (gui_config.min_width > gui_config.max_width) {
        gui_config.min_width = gui_config.max_width;
    }
    if (gui_config.min_height > gui_config.max_height) {
        gui_config.min_height = gui_config.max_height;
    }
}

void RectangleVisualizer::saveOriginalState() {
    original_placements = current_placements;
}

void RectangleVisualizer::generateInstance() {
    instance_generator = InstanceGenerator(
        gui_config.box_size,
        gui_config.min_width,
        gui_config.max_width,
        gui_config.min_height,
        gui_config.max_height
    );

    current_placements = instance_generator.generate_rectangles(gui_config.rect_count);
    problem = RectangleFittingProblem(gui_config.box_size, current_placements);

    saveOriginalState();

    setPlacements(problem.get_current_solution());
    setBoxLength(gui_config.box_size);

    // Reset solver states
    reset_relaxed_temperature();
    if (greedy_solver) {
        greedy_solver->reset_state();
    }
}

void RectangleVisualizer::generateRandomProblem() {
    generateInstance();
    reset_relaxed_temperature();
}

//Solver is called here
void RectangleVisualizer::runSolver() {
    //prepare multi threadding
    if (is_solving || solver_thread_active) {
        return;
    }

    if (solver_thread.joinable()) {
        solver_thread.join();
    }

    is_solving = true;
    solver_thread_active = true;
    solver_start_time = std::chrono::steady_clock::now();

    //Call solver depending on config
    solver_thread = std::thread([this]() {
        std::vector<RectanglePlacement> result;

        if (gui_config.solver_type == 0) { // Local Search
            if (gui_config.local_search_strategy == 0) {
                if (!geometry_solver) {
                    geometry_solver = std::make_unique<GeometryBasedNeighborhoodSolver>();
                }
                result = geometry_solver->solve(problem, gui_config.num_reruns, gui_config.max_rectangle_in_subproblem,gui_config.T);
            } else if (gui_config.local_search_strategy == 1) {
                if (!permutation_solver) {
                    permutation_solver = std::make_unique<RuleBasedNeighborhoodSolver>();
                }
                result = permutation_solver->solve(problem, gui_config.num_reruns, gui_config.max_rectangle_in_subproblem,gui_config.T);
            } else {
                if (!relaxed_geometry_solver) {
                    relaxed_geometry_solver = std::make_unique<RelaxedGeometryBasedNeighborhoodSolver>();
                }
                reset_relaxed_temperature();
                result = relaxed_geometry_solver->solve(problem, gui_config.num_reruns, gui_config.max_rectangle_in_subproblem,gui_config.T);
            }
        } else { // Greedy Solver
            if (!greedy_solver) {
                greedy_solver = std::make_unique<GreedySolver>();
            }

            greedy_solver->set_selection_strategy(gui_config.greedy_strategy);
            result = greedy_solver->solve(problem, gui_config.num_reruns, gui_config.max_rectangle_in_subproblem,gui_config.T);
        }

        {
            std::lock_guard<std::mutex> lock(solver_mutex);
            pending_result = result;
        }

        is_solving = false;
        solver_thread_active = false;
    });
}

void RectangleVisualizer::solveNextStep() {
    if (gui_config.T <= 0) gui_config.T = 1000;
    static std::vector<RectanglePlacement> prev_solution;
    static int prev_objective = 0;

    if (prev_solution.empty()) {
        prev_solution = problem.get_current_solution();
        prev_objective = problem.objective(prev_solution);
    }

    std::vector<RectanglePlacement> new_solution;

    if (gui_config.solver_type == 0) { // Local Search
        switch (gui_config.local_search_strategy) {
            case 0: // Geometry Based
                if (!geometry_solver) {
                    geometry_solver = std::make_unique<GeometryBasedNeighborhoodSolver>();
                }
                new_solution = geometry_solver->solve_one_step(problem, gui_config.T);
                break;

            case 1: // Permutation Based
                if (!permutation_solver) {
                    permutation_solver = std::make_unique<RuleBasedNeighborhoodSolver>();
                }
                new_solution = permutation_solver->solve_one_step(problem, gui_config.T);
                break;

            case 2: // Relaxed Geometry Based
                if (!relaxed_geometry_solver) {
                    relaxed_geometry_solver = std::make_unique<RelaxedGeometryBasedNeighborhoodSolver>();
                }
                new_solution = relaxed_geometry_solver->solve_one_step(problem, gui_config.T);
                updateT();
                break;
        }
    } else { // Greedy Solver
        if (!greedy_solver) {
            greedy_solver = std::make_unique<GreedySolver>();
        }
        greedy_solver->set_selection_strategy(gui_config.greedy_strategy);
        new_solution = greedy_solver->solve_one_step(problem, gui_config.T);
    }

    current_placements = new_solution;
    problem.set_current_solution(new_solution);

    int new_objective = problem.objective(new_solution);

    // Update visualization state
    g_changed_rect_idx = -1;

    if (new_solution.size() == prev_solution.size()) {
        for (size_t i = 0; i < new_solution.size(); i++) {
            if (new_solution[i].box_id != prev_solution[i].box_id ||
                new_solution[i].x != prev_solution[i].x ||
                new_solution[i].y != prev_solution[i].y ||
                new_solution[i].rotated != prev_solution[i].rotated) {
                g_changed_rect_idx = (int)i;

                // Auto-switch to the box containing the moved rectangle
                gui_config.view_all_boxes = false;
                // Store the actual box_id that we want to view
                gui_config.target_box_id = new_solution[i].box_id;

                break;
            }
        }
    } else if (new_solution.size() > prev_solution.size()) {
        // For Greedy step-by-step, highlight the newly placed rectangle
        g_changed_rect_idx = (int)new_solution.size() - 1;

        if (!new_solution.empty()) {
            gui_config.view_all_boxes = false;
            gui_config.target_box_id = new_solution.back().box_id;
        }
    }
    g_objective_improved = (new_objective < prev_objective);

    // Update for next step
    prev_solution = new_solution;
    prev_objective = new_objective;
}


void RectangleVisualizer::revertToOriginal() {
    if (!original_placements.empty()) {
        current_placements = original_placements;
        problem = RectangleFittingProblem(gui_config.box_size, original_placements);
        setPlacements(original_placements);
        reset_relaxed_temperature();

        // Reset greedy solver state if it exists
        if (greedy_solver) {
            greedy_solver->reset_state();
        }
    }
}

void RectangleVisualizer::runBenchmarkAsync() {
    if (is_benchmarking || benchmark_thread_active) {
        return;
    }

    if (benchmark_thread.joinable()) {
        benchmark_thread.join();
    }

    is_benchmarking = true;
    benchmark_thread_active = true;
    benchmark_start_time = std::chrono::steady_clock::now();

    benchmark_thread = std::thread([this]() {
        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Preparing benchmark...";
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "RECTANGLE PACKING SOLVER BENCHMARK" << std::endl;
        std::cout << "========================================\n" << std::endl;

        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Running benchmark...";
        }

        if (gui_config.benchmark_fast) {
            Benchmark::runNormal();
        } else {
            Benchmark::runHeavy();

        }

        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Benchmark complete!";
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "BENCHMARK COMPLETE" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));

        is_benchmarking = false;
        benchmark_thread_active = false;
    });
}

void RectangleVisualizer::runBenchmark() {
    runBenchmarkAsync();
}

void RectangleVisualizer::refreshBenchmarkFiles() {
    available_benchmark_files = BenchmarkLoader::getAvailableFiles();
    if (selected_file_index >= (int)available_benchmark_files.size()) {
        selected_file_index = -1;
    }
}

void RectangleVisualizer::loadBenchmarkSolution(const std::string& filepath) {
    auto loaded = BenchmarkLoader::load(filepath);
    if (loaded.has_value()) {
        current_placements = loaded->placements;
        setBoxLength(loaded->box_size);
        problem = RectangleFittingProblem(loaded->box_size, loaded->placements);

        // Update GUI config to match loaded solution
        gui_config.box_size = loaded->box_size;
        gui_config.rect_count = loaded->rect_count;
        gui_config.min_width = loaded->min_width;
        gui_config.max_width = loaded->max_width;
        gui_config.min_height = loaded->min_height;
        gui_config.max_height = loaded->max_height;

        saveOriginalState();

        std::cout << "Loaded solution: " << loaded->solver_name
                  << " | Boxes: " << loaded->boxes_used
                  << " | Utilization: " << loaded->utilization << "%"
                  << " | Objective: " << loaded->objective_value
                  << " | Time: " << loaded->solve_time << "s"
                  << std::endl;
    }
}

void RectangleVisualizer::pollEvents() {
    glfwPollEvents();
}

void RectangleVisualizer::render() {
    if (!initialized || !window) return;

    if (!is_solving && !solver_thread_active) {
        std::lock_guard<std::mutex> lock(solver_mutex);
        if (!pending_result.empty()) {
            current_placements = pending_result;
            problem.set_current_solution(pending_result);
            pending_result.clear();
        }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    int display_w, display_h;
    glfwGetFramebufferSize(static_cast<GLFWwindow*>(window), &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);

    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    std::vector<RectanglePlacement> display_placements = current_placements;

    std::unordered_set<int> used_boxes;
    for (const auto& placement : display_placements) used_boxes.insert(placement.box_id);
    std::vector<int> non_empty_boxes(used_boxes.begin(), used_boxes.end());
    std::sort(non_empty_boxes.begin(), non_empty_boxes.end());

    std::vector<int> boxes_to_display;
    if (gui_config.box_view_mode == VIEW_SINGLE) {
        if (!non_empty_boxes.empty()) {
            if (gui_config.current_box_view >= non_empty_boxes.size())
                gui_config.current_box_view = 0;

            if (gui_config.target_box_id != -1) {
                auto it = std::find(non_empty_boxes.begin(), non_empty_boxes.end(), gui_config.target_box_id);
                if (it != non_empty_boxes.end()) {
                    gui_config.current_box_view = std::distance(non_empty_boxes.begin(), it);
                }
                gui_config.target_box_id = -1;
            }

            boxes_to_display.push_back(non_empty_boxes[gui_config.current_box_view]);
        }
    } else {
        boxes_to_display = non_empty_boxes;
    }

    float box_spacing = 20.0f;
    int boxes_count = boxes_to_display.size();

    int cols = 1;
    int rows = 1;
    float local_scale = scale_factor;

    if (gui_config.box_view_mode == VIEW_ALL_HORIZONTAL) {
        cols = boxes_count;
        rows = 1;
    } else if (gui_config.box_view_mode == VIEW_ALL_FIT_SCREEN && boxes_count > 0) {
        cols = (int)std::ceil(std::sqrt((float)boxes_count));
        rows = (int)std::ceil((float)boxes_count / cols);

        float screen_ratio = (float)display_w / display_h;
        if (screen_ratio > 1.5f) {
            cols = std::min(boxes_count, 8);
            rows = (int)std::ceil((float)boxes_count / cols);
        }
    }

    float cell_width = box_length * local_scale;
    float cell_height = box_length * local_scale;

    if (gui_config.box_view_mode == VIEW_ALL_FIT_SCREEN && boxes_count > 0) {
        float avail_w = display_w - 450.0f;
        float avail_h = display_h - 150.0f;

        float max_cell_w = (avail_w - (cols - 1) * box_spacing) / cols;
        float max_cell_h = (avail_h - (rows - 1) * box_spacing) / rows;

        local_scale = std::min(max_cell_w / box_length, max_cell_h / box_length);
        local_scale = std::min(local_scale, scale_factor);
        local_scale = std::max(local_scale, 2.3f);

        cell_width = box_length * local_scale;
        cell_height = box_length * local_scale;
    }

    float total_width = cols * cell_width + (cols - 1) * box_spacing;
    float total_height = rows * cell_height + (rows - 1) * box_spacing;
    float start_x = (display_w - total_width) / 2.0f;
    float start_y = (display_h - total_height) / 2.0f;

    for (int display_index = 0; display_index < boxes_count; ++display_index) {
        int box_id = boxes_to_display[display_index];

        int row, col;
        if (gui_config.box_view_mode == VIEW_ALL_HORIZONTAL) {
            row = 0;
            col = display_index;
        } else if (gui_config.box_view_mode == VIEW_SINGLE) {
            row = 0;
            col = 0;
        } else {
            row = display_index / cols;
            col = display_index % cols;
        }

        float box_x = start_x + col * (cell_width + box_spacing);
        float box_y = start_y + row * (cell_height + box_spacing);

        draw_list->AddRectFilled(ImVec2(box_x, box_y), ImVec2(box_x + cell_width, box_y + cell_height), IM_COL32(40,40,40,255));
        draw_list->AddRect(ImVec2(box_x, box_y), ImVec2(box_x + cell_width, box_y + cell_height), IM_COL32(255,255,255,255), 0.0f, 0, 2.0f);

        std::string box_label = "Box " + std::to_string(box_id+1);
        draw_list->AddText(ImVec2(box_x + 5, box_y + 5), IM_COL32(255,255,255,255), box_label.c_str());
    }

    for (size_t idx = 0; idx < display_placements.size(); idx++) {
        const auto& placement = display_placements[idx];

        if (gui_config.box_view_mode == VIEW_SINGLE && !boxes_to_display.empty()) {
            if (placement.box_id != boxes_to_display[0]) continue;
        }

        int display_index = -1;
        for (int i = 0; i < boxes_to_display.size(); ++i) {
            if (boxes_to_display[i] == placement.box_id) {
                display_index = i;
                break;
            }
        }
        if (display_index == -1) continue;

        int row, col;
        if (gui_config.box_view_mode == VIEW_ALL_HORIZONTAL) {
            row = 0;
            col = display_index;
        } else if (gui_config.box_view_mode == VIEW_SINGLE) {
            row = 0;
            col = 0;
        } else {
            row = display_index / cols;
            col = display_index % cols;
        }

        float box_x = start_x + col * (cell_width + box_spacing);
        float box_y = start_y + row * (cell_height + box_spacing);

        float rect_w = placement.get_actual_width() * local_scale;
        float rect_h = placement.get_actual_height() * local_scale;
        float rect_x = box_x + placement.x * local_scale;
        float rect_y = box_y + placement.y * local_scale;

        ImU32 color = IM_COL32(128, 128, 128, 150);

        if ((int)idx == g_changed_rect_idx) {
            color = g_objective_improved
                ? IM_COL32(255, 255, 0, 200)
                : IM_COL32(0, 0, 255, 200);
        }

        draw_list->AddRectFilled(ImVec2(rect_x, rect_y), ImVec2(rect_x + rect_w, rect_y + rect_h), color);
        draw_list->AddRect(ImVec2(rect_x, rect_y), ImVec2(rect_x + rect_w, rect_y + rect_h), IM_COL32(255,255,255,255),0.0f,0,1.5f);

        if (rect_w > 20 && rect_h > 15) {
            std::string size_label = std::to_string(placement.width) + "x" + std::to_string(placement.height);
            if (placement.rotated) size_label += " R";

            ImVec2 text_size = ImGui::CalcTextSize(size_label.c_str());
            float text_x = rect_x + rect_w/2 - text_size.x/2;
            float text_y = rect_y + rect_h/2 - text_size.y/2;

            draw_list->AddRectFilled(ImVec2(text_x-2,text_y-1), ImVec2(text_x+text_size.x+2,text_y+text_size.y+1), IM_COL32(0,0,0,128));
            draw_list->AddText(ImVec2(text_x,text_y), IM_COL32(255,255,255,255), size_label.c_str());
        }
    }

    // Left Window: Problem Configuration
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(430, display_h / 2 + 25), ImGuiCond_FirstUseEver);
    ImGui::Begin("Problem Configuration", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::CollapsingHeader("Instance Generation", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderInt("Rectangle Count", &gui_config.rect_count, 10, 1000);
        ImGui::SliderInt("Box Size (L)", &gui_config.box_size, 10, 80);
        if (ImGui::IsItemDeactivatedAfterEdit()) updateMaxSizeLimits();
        ImGui::SliderInt("Min Width", &gui_config.min_width, 5, box_length);
        ImGui::SliderInt("Max Width", &gui_config.max_width, std::min(20,box_length), box_length);
        ImGui::SliderInt("Min Height", &gui_config.min_height, 5, box_length);
        ImGui::SliderInt("Max Height", &gui_config.max_height, std::min(10,box_length), box_length);

        ImGui::Spacing();
        if (ImGui::Button("Generate New Problem", ImVec2(-1, 0))) {
            generateRandomProblem();
        }
        ImGui::Separator();
    }

    if (ImGui::CollapsingHeader("Solver Configuration", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Solver Type:");
        const char* solver_types[] = { "Local Search", "Greedy" };
        ImGui::Combo("Solver Type", &gui_config.solver_type, solver_types, IM_ARRAYSIZE(solver_types));

        if (gui_config.solver_type == 0) {
            ImGui::Text("Local Search Strategy:");
            const char* local_strategies[] = { "Geometry Based", "Permutation Based", "Relaxed Geometry Based"};
            ImGui::Combo("Strategy", &gui_config.local_search_strategy, local_strategies, IM_ARRAYSIZE(local_strategies));
        } else {
            ImGui::Text("Greedy Strategy:");
            const char* greedy_strategies[] = { "Biggest First", "Smallest First", "Best Fit" };
            ImGui::Combo("Strategy", &gui_config.greedy_strategy, greedy_strategies, IM_ARRAYSIZE(greedy_strategies));
        }

        // Show "Solve One Step" button for both Local Search and Greedy
        ImGui::Spacing();
        ImGui::BeginDisabled(is_solving || is_benchmarking);
        if (ImGui::Button("Solve One Step", ImVec2(-1, 0))) {
            solveNextStep();
        }
        ImGui::EndDisabled();
        ImGui::Spacing();


        ImGui::SliderInt("Temperature (T)", &gui_config.T, 1, 10000);
        //ImGui::SliderInt("Reruns", &gui_config.num_reruns, 1, 10);
        //ImGui::SliderInt("Max Subproblem", &gui_config.max_rectangle_in_subproblem, 10, 200);

        //ImGui::Spacing();

        if (is_solving && !is_benchmarking) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - solver_start_time);

            std::ostringstream time_ss;
            time_ss << std::setw(2) << std::setfill('0') << (elapsed.count() / 60) << ":"
                    << std::setw(2) << std::setfill('0') << (elapsed.count() % 60);

            int dot_count = (int)(ImGui::GetTime() * 2.0) % 4;
            std::string calculating_text = "Calculating";
            for (int i = 0; i < dot_count; i++) {
                calculating_text += ".";
            }

            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "%s", calculating_text.c_str());
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "(%s)", time_ss.str().c_str());
            ImGui::Spacing();
        }

        ImGui::BeginDisabled(is_solving || is_benchmarking);
        if (ImGui::Button("Run Solver", ImVec2(-1, 45))) {
            gui_config.T = 1000;
            runSolver();
        }
        ImGui::EndDisabled();

        ImGui::Spacing();

        ImGui::BeginDisabled(is_solving || is_benchmarking);
        if (ImGui::Button("Revert to Original", ImVec2(-1, 45))) {
            revertToOriginal();
        }
        ImGui::EndDisabled();
        ImGui::Separator();
    }

    ImGui::End();

    // Right Window: Visualization & Results
    ImGui::SetNextWindowPos(ImVec2(display_w - 440, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(430, display_h / 2 - 20), ImGuiCond_FirstUseEver);
    ImGui::Begin("Visualization & Results", nullptr, ImGuiWindowFlags_NoSavedSettings);

    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Box Viewing:");
        const char* view_modes[] = {
            "Single Box",
            "All Boxes (Horizontal)",
            "All Boxes (Fit Screen)"
        };

        ImGui::Combo("Box View Mode", &gui_config.box_view_mode, view_modes, IM_ARRAYSIZE(view_modes));

        if (gui_config.box_view_mode == VIEW_SINGLE && !non_empty_boxes.empty()) {
            int max_index = (int)non_empty_boxes.size() - 1;
            if (gui_config.current_box_view > max_index) {
                gui_config.current_box_view = 0;
            }

            ImGui::SliderInt("Box Index", &gui_config.current_box_view, 0, max_index);

            int actual_box_id = non_empty_boxes[gui_config.current_box_view];
            ImGui::Text("Viewing Box %d (ID: %d)", gui_config.current_box_view + 1, actual_box_id);
        } else if (gui_config.box_view_mode == VIEW_SINGLE && non_empty_boxes.empty()) {
            ImGui::Text("No boxes with rectangles to display");
        }
        ImGui::Spacing();
        ImGui::Separator();
    }

    if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Rectangles: %zu", display_placements.size());
        ImGui::Text("Boxes Used: %zu", non_empty_boxes.size());

        if (!display_placements.empty()) {
            float total_area=0, used_area=0;
            if (gui_config.box_view_mode != VIEW_SINGLE) {
                total_area = non_empty_boxes.size()*box_length*box_length;
                for (auto& r:display_placements) used_area += r.width*r.height;
            } else {
                if (!boxes_to_display.empty()) {
                    total_area = box_length*box_length;
                    for (auto& r:display_placements) if (r.box_id==boxes_to_display[0]) used_area+=r.width*r.height;
                }
            }
            if (total_area>0) {
                float utilization = (used_area/total_area)*100.0f;
                ImGui::Text("Utilization: %.1f%%", utilization);
            }
        }
        ImGui::Spacing();
        ImGui::Separator();
    }

    if (ImGui::CollapsingHeader("Benchmark", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Configuration:");
        static bool benchmark_fast_mode = true;
        ImGui::Checkbox("Fast Benchmark Mode", &benchmark_fast_mode);
        gui_config.benchmark_fast = benchmark_fast_mode;

        ImGui::BeginDisabled(is_benchmarking || is_solving);
        if (ImGui::Button("Run Benchmark", ImVec2(-1, 0))) {
            runBenchmarkAsync();
        }
        ImGui::EndDisabled();

        if (is_benchmarking) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - benchmark_start_time);

            std::ostringstream time_ss;
            time_ss << std::setw(2) << std::setfill('0') << (elapsed.count() / 60) << ":"
                    << std::setw(2) << std::setfill('0') << (elapsed.count() % 60);

            int dot_count = (int)(ImGui::GetTime() * 2.0) % 4;
            std::string benchmark_text = "Benchmarking";
            for (int i = 0; i < dot_count; i++) {
                benchmark_text += ".";
            }

            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "%s", benchmark_text.c_str());
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "(%s)", time_ss.str().c_str());

            std::lock_guard<std::mutex> lock(benchmark_mutex);
            ImGui::Text("Status: %s", benchmark_status.c_str());
        }

        ImGui::Spacing();
        ImGui::Separator();

        ImGui::Text("Load Solution:");
        if (ImGui::Button("Refresh File List", ImVec2(-1, 0))) {
            refreshBenchmarkFiles();
        }

        if (available_benchmark_files.empty()) {
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No benchmark XML files found");
        } else {
            ImGui::Text("Available files: %zu", available_benchmark_files.size());

            ImGui::BeginChild("FileList", ImVec2(0, 120), true);
            for (int i = 0; i < (int)available_benchmark_files.size(); i++) {
                bool is_selected = (selected_file_index == i);
                if (ImGui::Selectable(available_benchmark_files[i].c_str(), is_selected)) {
                    selected_file_index = i;
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndChild();

            ImGui::BeginDisabled(selected_file_index < 0 || is_solving || is_benchmarking);
            if (ImGui::Button("Load Selected Solution", ImVec2(-1, 0))) {
                if (selected_file_index >= 0 && selected_file_index < (int)available_benchmark_files.size()) {
                    loadBenchmarkSolution(available_benchmark_files[selected_file_index]);
                }
            }
            ImGui::EndDisabled();

            if (selected_file_index >= 0 && selected_file_index < (int)available_benchmark_files.size()) {
                ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                                "Selected: %s", available_benchmark_files[selected_file_index].c_str());
            }
        }
    }

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(static_cast<GLFWwindow*>(window));
}

bool RectangleVisualizer::shouldClose() const {
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(window));
}