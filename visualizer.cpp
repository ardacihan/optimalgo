#include "visualizer.h"
#include <iostream>
#include <algorithm>
#include "imgui_internal.h"

RectangleVisualizer::RectangleVisualizer(int width, int height)
    : box_length(15), scale_factor(1.0f), offset{50.0f, 50.0f}, initialized(false),
      instance_generator(
          gui_config.box_size,
          gui_config.min_width,
          gui_config.max_width,
          gui_config.min_height,
          gui_config.max_height
      ),
      problem(gui_config.box_size, std::vector<RectanglePlacement>()) {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // Generate random problem at startup
    generateRandomProblem();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    window = glfwCreateWindow(width, height, "Rectangle Packing Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(static_cast<GLFWwindow *>(window));
    glfwSwapInterval(1);
}

RectangleVisualizer::~RectangleVisualizer() {
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
        std::cerr << "No window created" << std::endl;
        return false;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(static_cast<GLFWwindow*>(window), true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend" << std::endl;
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        std::cerr << "Failed to initialize ImGui OpenGL backend" << std::endl;
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
    // Ensure max dimensions don't exceed box size
    if (gui_config.max_width > box_length) {
        gui_config.max_width = box_length;
    }
    if (gui_config.max_height > box_length) {
        gui_config.max_height = box_length;
    }
    // Ensure min dimensions don't exceed max dimensions
    if (gui_config.min_width > gui_config.max_width) {
        gui_config.min_width = gui_config.max_width;
    }
    if (gui_config.min_height > gui_config.max_height) {
        gui_config.min_height = gui_config.max_height;
    }
}

void RectangleVisualizer::generateInstance() {
    // Create InstanceGenerator with GUI parameters
    instance_generator = InstanceGenerator(
        gui_config.box_size,
        gui_config.min_width,
        gui_config.max_width,
        gui_config.min_height,
        gui_config.max_height
    );

    current_placements = instance_generator.generate_rectangles(gui_config.rect_count);
    problem = RectangleFittingProblem(gui_config.box_size, current_placements);

    setPlacements(problem.get_current_solution());
    setBoxLength(gui_config.box_size);
}

void RectangleVisualizer::generateRandomProblem() {
    generateInstance();
}

void RectangleVisualizer::runSolver() {
    solver.solve(problem, gui_config.num_reruns,gui_config.max_rectangle_in_subproblem);
    current_placements = problem.get_current_solution();
}

void RectangleVisualizer::solveNextStep() {
    solver.solve_one_step(problem);
    current_placements = problem.get_current_solution();
}

void RectangleVisualizer::pollEvents() {
    glfwPollEvents();
}

void RectangleVisualizer::render() {
    if (!initialized || !window) return;

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

    // Collect non-empty boxes
    std::unordered_set<int> used_boxes;
    for (const auto& placement : display_placements) used_boxes.insert(placement.box_id);
    std::vector<int> non_empty_boxes(used_boxes.begin(), used_boxes.end());
    std::sort(non_empty_boxes.begin(), non_empty_boxes.end());

    std::vector<int> boxes_to_display;
    if (gui_config.view_all_boxes) {
        boxes_to_display = non_empty_boxes;
    } else {
        if (!non_empty_boxes.empty()) {
            if (gui_config.current_box_view >= non_empty_boxes.size()) gui_config.current_box_view = 0;
            boxes_to_display.push_back(non_empty_boxes[gui_config.current_box_view]);
        }
    }

    float box_spacing = 20.0f;
    int boxes_count = boxes_to_display.size();
    float total_width = boxes_count * (box_length * scale_factor) + (boxes_count - 1) * box_spacing;
    float start_x = (display_w - total_width) / 2.0f; // Original centering logic

    // Draw boxes
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

    // Colors for boxes
    ImU32 box_colors[] = {IM_COL32(65,105,225,200),IM_COL32(220,20,60,200),IM_COL32(50,205,50,200),
                          IM_COL32(255,140,0,200),IM_COL32(148,0,211,200),IM_COL32(255,215,0,200),
                          IM_COL32(0,206,209,200),IM_COL32(255,99,71,200)};

    // Draw rectangles
    for (const auto& placement : display_placements) {
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

        ImU32 color = box_colors[placement.box_id % 8];

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

    // GUI window
    ImGui::SetNextWindowPos(ImVec2(20,20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400,500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Rectangle Packing Controls");

    ImGui::Text("Instance Generator Parameters");
    ImGui::Separator();
    ImGui::SliderInt("Rectangle Count",&gui_config.rect_count,1,1000);
    ImGui::SliderInt("Box Size (L)",&gui_config.box_size,10,20);
    if (ImGui::IsItemDeactivatedAfterEdit()) updateMaxSizeLimits();
    ImGui::SliderInt("Min Width",&gui_config.min_width,1,box_length);
    ImGui::SliderInt("Max Width",&gui_config.max_width,gui_config.min_width,box_length);
    ImGui::SliderInt("Min Height",&gui_config.min_height,1,box_length);
    ImGui::SliderInt("Max Height",&gui_config.max_height,gui_config.min_height,box_length);

    ImGui::Separator();
    ImGui::Text("Box Viewing:");
    ImGui::Checkbox("View All Boxes",&gui_config.view_all_boxes);
    if (!gui_config.view_all_boxes && non_empty_boxes.size()>0) {
        if (gui_config.current_box_view>=non_empty_boxes.size()) gui_config.current_box_view=0;
        ImGui::SliderInt("View Box",&gui_config.current_box_view,0,(int)non_empty_boxes.size()-1);
        ImGui::Text("Viewing Box %d of %zu", gui_config.current_box_view+1, non_empty_boxes.size());
    } else if (!gui_config.view_all_boxes && non_empty_boxes.empty()) {
        ImGui::Text("No boxes with rectangles to display");
    }

    ImGui::Separator();
    if (ImGui::Button("Generate Random Problem")) generateRandomProblem();
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        current_placements.clear();
        gui_config.current_box_view=0;
        gui_config.show_solver_steps=false;
        problem = RectangleFittingProblem(gui_config.box_size,std::vector<RectanglePlacement>());
    }

    ImGui::Separator();
    ImGui::Text("Solver Controls:");
    ImGui::SliderInt("Number of Reruns",&gui_config.num_reruns,1,100);
    ImGui::SliderInt("Max Rectangles in Subproblem",&gui_config.max_rectangle_in_subproblem,10,200);
    if (ImGui::Button("Solve Next Step")) solveNextStep();
    ImGui::SameLine();
    if (ImGui::Button("Run Solver")) runSolver();

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

    ImGui::End();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(static_cast<GLFWwindow*>(window));
}


bool RectangleVisualizer::shouldClose() const {
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(window));
}