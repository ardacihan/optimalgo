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
    double cooling_rate = 0.94;
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
}

void RectangleVisualizer::generateRandomProblem() {
    generateInstance();
    reset_relaxed_temperature();
}

void RectangleVisualizer::runSolver() {
    if (is_solving || solver_thread_active) {
        return;
    }

    if (solver_thread.joinable()) {
        solver_thread.join();
    }

    is_solving = true;
    solver_thread_active = true;
    solver_start_time = std::chrono::steady_clock::now();

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

    switch (gui_config.local_search_strategy) {
        case 0: // Geometry Based
            if (!geometry_solver) {
                geometry_solver = std::make_unique<GeometryBasedNeighborhoodSolver>();
            }
            new_solution = geometry_solver->solve_one_step(problem, gui_config.T);
            current_placements = new_solution;
            problem.set_current_solution(new_solution);
            break;

        case 1: // Permutation Based
            if (!permutation_solver) {
                permutation_solver = std::make_unique<RuleBasedNeighborhoodSolver>();
            }
            new_solution = permutation_solver->solve_one_step(problem, gui_config.T);
            current_placements = new_solution;
            problem.set_current_solution(new_solution);
            break;

        case 2: // Relaxed Geometry Based
            if (!relaxed_geometry_solver) {
                relaxed_geometry_solver = std::make_unique<RelaxedGeometryBasedNeighborhoodSolver>();
            }
            new_solution = relaxed_geometry_solver->solve_one_step(problem, gui_config.T);
            current_placements = new_solution;
            problem.set_current_solution(new_solution);
            updateT();
            break;
    }

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

        bool quick_mode = true;

        std::cout << "\n========================================" << std::endl;
        std::cout << "RECTANGLE PACKING SOLVER BENCHMARK" << std::endl;
        std::cout << "========================================\n" << std::endl;

        std::cout << "NOTE: This benchmark tests 6 solvers on each instance:" << std::endl;
        std::cout << "  1. Geometry-Based Solver" << std::endl;
        std::cout << "  2. Rule-Based Solver" << std::endl;
        std::cout << "  3. Relaxed Geometry Solver" << std::endl;
        std::cout << "  4. Greedy (Biggest First)" << std::endl;
        std::cout << "  5. Greedy (Smallest First)" << std::endl;
        std::cout << "  6. Greedy (Area Descending)" << std::endl;
        std::cout << std::endl;

        // Use fixed output filename
        std::string output_filename = "benchmark_results.txt";
        BenchmarkRunner runner(output_filename);

        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Setting up configurations...";
        }

        if (gui_config.benchmark_fast) {
            std::cout << "=== QUICK BENCHMARK MODE ===\n" << std::endl;

            // Reduced for faster testing
            runner.add_config({1, 500, 10, 20, 10, 20, 80});
            runner.add_config({1, 500, 10, 20, 10, 20, 80});
            runner.add_config({1, 500, 10, 40, 10, 40, 80});
            runner.add_config({1, 500, 10, 40, 10, 40, 100});
            runner.add_config({1, 1000, 10, 20, 10, 20, 80});
            runner.add_config({1, 1000, 10, 20, 10, 20, 80});
            runner.add_config({1, 1000, 10, 40, 10, 40, 80});
            runner.add_config({1, 1000, 10, 40, 10, 40, 100});
        } else {
            std::cout << "=== FULL BENCHMARK MODE ===\n" << std::endl;

            runner.add_config({3, 100, 5, 10, 5, 10, 20});
            runner.add_config({3, 200, 5, 15, 5, 15, 30});
            runner.add_config({2, 300, 10, 20, 10, 20, 50});
        }

        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Running benchmark...";
        }

        runner.run_benchmark();

        {
            std::lock_guard<std::mutex> lock(benchmark_mutex);
            benchmark_status = "Benchmark complete!";
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "BENCHMARK COMPLETE" << std::endl;
        std::cout << "========================================\n" << std::endl;

        // Wait a moment so the user can see the completion message
        std::this_thread::sleep_for(std::chrono::seconds(2));

        is_benchmarking = false;
        benchmark_thread_active = false;
    });
}

void RectangleVisualizer::runBenchmark() {
    runBenchmarkAsync();
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
    if (gui_config.view_all_boxes) {
        boxes_to_display = non_empty_boxes;
    } else {
        if (!non_empty_boxes.empty()) {
            // Use index-based selection
            if (gui_config.current_box_view >= non_empty_boxes.size()) {
                gui_config.current_box_view = 0;
            }

            // If we have a target box ID from solve_one_step, find its index
            if (gui_config.target_box_id != -1) {
                auto it = std::find(non_empty_boxes.begin(), non_empty_boxes.end(), gui_config.target_box_id);
                if (it != non_empty_boxes.end()) {
                    gui_config.current_box_view = std::distance(non_empty_boxes.begin(), it);
                }
                gui_config.target_box_id = -1; // Reset after using
            }

            boxes_to_display.push_back(non_empty_boxes[gui_config.current_box_view]);
        }
    }

    float box_spacing = 20.0f;
    int boxes_count = boxes_to_display.size();
    float total_width = boxes_count * (box_length * scale_factor) + (boxes_count - 1) * box_spacing;
    float start_x = (display_w - total_width) / 2.0f;

    for (int display_index = 0; display_index < boxes_count; ++display_index) {
        int box_id = boxes_to_display[display_index];
        float box_x = start_x + display_index * (box_length * scale_factor + box_spacing);
        float box_y = offset.y;
        float box_size = box_length * scale_factor;

        draw_list->AddRectFilled(ImVec2(box_x, box_y), ImVec2(box_x + box_size, box_y + box_size), IM_COL32(40,40,40,255));
        draw_list->AddRect(ImVec2(box_x, box_y), ImVec2(box_x + box_size, box_y + box_size), IM_COL32(255,255,255,255), 0.0f, 0, 2.0f);

        std::string box_label = "Box " + std::to_string(box_id+1);
        if (!gui_config.view_all_boxes) box_label += " (Viewing)";
        draw_list->AddText(ImVec2(box_x + 5, box_y + 5), IM_COL32(255,255,255,255), box_label.c_str());
    }

    for (size_t idx = 0; idx < display_placements.size(); idx++) {
        const auto& placement = display_placements[idx];
        if (!gui_config.view_all_boxes) {
            int current_box = boxes_to_display.empty() ? -1 : boxes_to_display[0];
            if (placement.box_id != current_box) continue;
        }

        int display_index = -1;
        for (int i=0;i<boxes_to_display.size();++i) if (boxes_to_display[i]==placement.box_id) display_index=i;
        if (display_index==-1) continue;

        float box_x = start_x + display_index * (box_length * scale_factor + box_spacing);

        float rect_w = placement.get_actual_width() * scale_factor;
        float rect_h = placement.get_actual_height() * scale_factor;
        float rect_x = box_x + placement.x * scale_factor;
        float rect_y = offset.y + placement.y * scale_factor;

        ImU32 color = IM_COL32(128, 128, 128, 150);

        if ((int)idx == g_changed_rect_idx) {
            color = g_objective_improved
                ? IM_COL32(255, 255, 0, 200)   // Yellow fill if improved
                : IM_COL32(0, 0, 255, 200);    // Blue fill if worse
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

    ImGui::SetNextWindowPos(ImVec2(20,20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400,600), ImGuiCond_FirstUseEver);
    ImGui::Begin("Rectangle Packing Controls");

    ImGui::Text("Instance Generator Parameters");
    ImGui::Separator();
    ImGui::SliderInt("Rectangle Count",&gui_config.rect_count,10,1000);
    ImGui::SliderInt("Box Size (L)",&gui_config.box_size,10,80);
    if (ImGui::IsItemDeactivatedAfterEdit()) updateMaxSizeLimits();
    ImGui::SliderInt("Min Width",&gui_config.min_width,5,box_length);
    ImGui::SliderInt("Max Width",&gui_config.max_width,std::min(20,box_length),box_length);
    ImGui::SliderInt("Min Height",&gui_config.min_height,5,box_length);
    ImGui::SliderInt("Max Height",&gui_config.max_height,std::min(10,box_length),box_length);
    if (ImGui::Button("Generate new problem")) generateRandomProblem();
    ImGui::Separator();
    ImGui::Text("Box Viewing:");
    ImGui::Checkbox("View All Boxes",&gui_config.view_all_boxes);
    if (!gui_config.view_all_boxes && non_empty_boxes.size()>0) {
        // Use index-based slider (0 to num_boxes-1)
        int max_index = (int)non_empty_boxes.size() - 1;
        if (gui_config.current_box_view > max_index) {
            gui_config.current_box_view = 0;
        }

        ImGui::SliderInt("Box Index", &gui_config.current_box_view, 0, max_index);

        // Show the actual box ID for clarity
        int actual_box_id = non_empty_boxes[gui_config.current_box_view];
        ImGui::Text("Viewing Box %d (ID: %d)", gui_config.current_box_view + 1, actual_box_id);
    } else if (!gui_config.view_all_boxes && non_empty_boxes.empty()) {
        ImGui::Text("No boxes with rectangles to display");
    }

    ImGui::Separator();
    ImGui::Text("Solver Configuration:");

    const char* solver_types[] = { "Local Search", "Greedy" };
    ImGui::Combo("Solver Type", &gui_config.solver_type, solver_types, IM_ARRAYSIZE(solver_types));

    if (gui_config.solver_type == 0) {
        const char* local_strategies[] = { "Geometry Based", "Permutation Based", "Relaxed Geometry Based"};
        ImGui::Combo("Local Search Strategy", &gui_config.local_search_strategy, local_strategies, IM_ARRAYSIZE(local_strategies));
        ImGui::BeginDisabled(is_solving || is_benchmarking);
        if (ImGui::Button("Solve One Step")) {
            solveNextStep();
        }
        ImGui::EndDisabled();
    } else {
        const char* greedy_strategies[] = { "Biggest First", "Smallest First", "Best Fit" };
        ImGui::Combo("Greedy Strategy", &gui_config.greedy_strategy, greedy_strategies, IM_ARRAYSIZE(greedy_strategies));
    }

    ImGui::Separator();

    ImGui::BeginDisabled(is_solving || is_benchmarking);
    if (ImGui::Button("Run Solver")) {
        gui_config.T = 1000;
        runSolver();
    }

    ImGui::SameLine();
    if (ImGui::Button("Revert")) revertToOriginal();
    ImGui::EndDisabled();

    if (is_solving && !is_benchmarking) {
        ImGui::Separator();

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
    }

    ImGui::Separator();
    ImGui::Text("Statistics:");
    ImGui::Text("Rectangles: %zu", display_placements.size());
    ImGui::Text("Boxes Used: %zu", non_empty_boxes.size());

    if (!display_placements.empty()) {
        float total_area=0, used_area=0;
        if (gui_config.view_all_boxes) {
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

    ImGui::Separator();

    // Benchmark section
    ImGui::Text("Benchmark Configuration:");
    static bool benchmark_fast_mode = true;
    ImGui::Checkbox("Fast Benchmark Mode", &benchmark_fast_mode);
    gui_config.benchmark_fast = benchmark_fast_mode;

    ImGui::BeginDisabled(is_benchmarking || is_solving);
    if (ImGui::Button("Run Benchmark")) {
        runBenchmarkAsync();
    }
    ImGui::EndDisabled();

    if (is_benchmarking) {
        ImGui::Separator();

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

        // Show status message
        std::lock_guard<std::mutex> lock(benchmark_mutex);
        ImGui::Text("Status: %s", benchmark_status.c_str());
    }



    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(static_cast<GLFWwindow*>(window));
}

bool RectangleVisualizer::shouldClose() const {
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(window));
}