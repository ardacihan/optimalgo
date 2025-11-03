#include "visualizer.h"
#include <iostream>
#include <algorithm>

RectangleVisualizer::RectangleVisualizer(int width, int height)
    : box_length(100), scale_factor(1.0f), offset{50.0f, 50.0f}, initialized(false) {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    instance_generator =  InstanceGenerator(
        gui_config.box_size,
        gui_config.min_width,
        gui_config.max_width,
        gui_config.min_height,
        gui_config.max_height);

    current_placements = instance_generator.generate_rectangles(10);

    problem = RectangleFittingProblem(gui_config.box_size, current_placements);

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
        std::cerr << "Failed to initialize ImGui OpenGL backend" << std::endl;  // Fixed the error here
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

    current_placements = instance_generator.generate_rectangles(gui_config.rect_count) ;
    problem = RectangleFittingProblem(
        gui_config.box_size,current_placements);

    // Generate rectangles using your method
    //auto placements = instance_generator->generate_rectangles(gui_config.rect_count);

    // Use better initialization from InstanceGenerator
   // auto better_placements = instance_generator->create_better_initial_solution(placements, gui_config.box_size);

    setPlacements(problem.get_current_solution());
    setBoxLength(gui_config.box_size);
}

void RectangleVisualizer::runSolver() {
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

    // Use current placements (solver visualization temporarily disabled)
    std::vector<RectanglePlacement> display_placements = current_placements;

    // Calculate number of boxes needed
    int num_boxes = 1;
    if (!display_placements.empty()) {
        int max_box_id = 0;
        for (const auto& placement : display_placements) {
            if (placement.box_id > max_box_id) max_box_id = placement.box_id;
        }
        num_boxes = max_box_id + 1;
    }

    // Determine which boxes to display
    std::vector<int> boxes_to_display;
    if (gui_config.view_all_boxes) {
        // Display all boxes
        for (int i = 0; i < num_boxes; ++i) {
            boxes_to_display.push_back(i);
        }
    } else {
        // Display only the selected box
        if (gui_config.current_box_view < num_boxes) {
            boxes_to_display.push_back(gui_config.current_box_view);
        } else if (num_boxes > 0) {
            gui_config.current_box_view = 0;
            boxes_to_display.push_back(0);
        }
    }

    // Draw bounding boxes
    float box_spacing = 20.0f;
    int boxes_count = boxes_to_display.size();
    float total_width = boxes_count * (box_length * scale_factor) + (boxes_count - 1) * box_spacing;
    float start_x = (display_w - total_width) / 2.0f;

    for (int display_index = 0; display_index < boxes_count; ++display_index) {
        int box_id = boxes_to_display[display_index];
        float box_x = start_x + display_index * (box_length * scale_factor + box_spacing);
        float box_y = offset.y;
        float box_size = box_length * scale_factor;

        // Box background
        draw_list->AddRectFilled(
            ImVec2(box_x, box_y),
            ImVec2(box_x + box_size, box_y + box_size),
            IM_COL32(40, 40, 40, 255)
        );

        // Box border
        draw_list->AddRect(
            ImVec2(box_x, box_y),
            ImVec2(box_x + box_size, box_y + box_size),
            IM_COL32(255, 255, 255, 255),
            0.0f, 0, 2.0f
        );

        // Box label
        std::string box_label = "Box " + std::to_string(box_id + 1);
        if (!gui_config.view_all_boxes) {
            box_label += " (Viewing)";
        }
        draw_list->AddText(ImVec2(box_x + 5, box_y + 5), IM_COL32(255, 255, 255, 255), box_label.c_str());
    }

    // Draw rectangles with colors based on box_id
    ImU32 box_colors[] = {
        IM_COL32(65, 105, 225, 200),  // Blue
        IM_COL32(220, 20, 60, 200),   // Red
        IM_COL32(50, 205, 50, 200),   // Green
        IM_COL32(255, 140, 0, 200),   // Orange
        IM_COL32(148, 0, 211, 200),   // Purple
        IM_COL32(255, 215, 0, 200),   // Yellow
        IM_COL32(0, 206, 209, 200),   // Teal
        IM_COL32(255, 99, 71, 200)    // Tomato
    };

    for (const auto& placement : display_placements) {
        // Skip if we're viewing a specific box and this rectangle isn't in it
        if (!gui_config.view_all_boxes && placement.box_id != gui_config.current_box_view) {
            continue;
        }

        // Find the display index for this box
        int display_index = -1;
        for (int i = 0; i < boxes_to_display.size(); ++i) {
            if (boxes_to_display[i] == placement.box_id) {
                display_index = i;
                break;
            }
        }
        if (display_index == -1) continue;

        float box_x = start_x + display_index * (box_length * scale_factor + box_spacing);

        float rect_x = box_x + placement.x * scale_factor;
        float rect_y = offset.y + placement.y * scale_factor;
        float rect_w = placement.width * scale_factor;
        float rect_h = placement.height * scale_factor;

        ImU32 color = box_colors[placement.box_id % 8];

        // Draw rectangle
        draw_list->AddRectFilled(
            ImVec2(rect_x, rect_y),
            ImVec2(rect_x + rect_w, rect_y + rect_h),
            color
        );

        // Rectangle border
        draw_list->AddRect(
            ImVec2(rect_x, rect_y),
            ImVec2(rect_x + rect_w, rect_y + rect_h),
            IM_COL32(255, 255, 255, 255),
            0.0f, 0, 1.5f
        );

        // Size label
        if (rect_w > 20 && rect_h > 15) {
            std::string size_label = std::to_string(placement.width) + "x" + std::to_string(placement.height);
            if (placement.rotated) {
                size_label += " R";
            }
            ImVec2 text_size = ImGui::CalcTextSize(size_label.c_str());
            float text_x = rect_x + rect_w / 2 - text_size.x / 2;
            float text_y = rect_y + rect_h / 2 - text_size.y / 2;

            draw_list->AddRectFilled(
                ImVec2(text_x - 2, text_y - 1),
                ImVec2(text_x + text_size.x + 2, text_y + text_size.y + 1),
                IM_COL32(0, 0, 0, 128)
            );

            draw_list->AddText(ImVec2(text_x, text_y), IM_COL32(255, 255, 255, 255), size_label.c_str());
        }
    }

    // GUI Window
    ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Rectangle Packing Controls");

    ImGui::Text("Instance Generator Parameters");
    ImGui::Separator();

    // Parameters that match your InstanceGenerator constructor
    ImGui::SliderInt("Rectangle Count", &gui_config.rect_count, 1, 1000);
    ImGui::SliderInt("Box Size (L)", &gui_config.box_size, 50, 200);

    // Update max limits when box size changes
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        updateMaxSizeLimits();
    }

    ImGui::SliderInt("Min Width", &gui_config.min_width, 1, box_length);
    ImGui::SliderInt("Max Width", &gui_config.max_width, gui_config.min_width, box_length);
    ImGui::SliderInt("Min Height", &gui_config.min_height, 1, box_length);
    ImGui::SliderInt("Max Height", &gui_config.max_height, gui_config.min_height, box_length);

    ImGui::Separator();

    // Box viewing controls
    ImGui::Text("Box Viewing:");
    ImGui::Checkbox("View All Boxes", &gui_config.view_all_boxes);

    if (!gui_config.view_all_boxes && num_boxes > 0) {
        ImGui::SliderInt("View Box", &gui_config.current_box_view, 0, num_boxes - 1);
        ImGui::Text("Viewing Box %d of %d", gui_config.current_box_view + 1, num_boxes);
    }

    ImGui::Separator();

    if (ImGui::Button("Generate (Better Init)")) {
        generateInstance();
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        current_placements.clear();
        gui_config.current_box_view = 0;
        gui_config.show_solver_steps = false;
    }

    ImGui::Separator();

    // Temporarily disable solver controls
    ImGui::Text("Solver Controls: (Coming Soon)");
    //ImGui::BeginDisabled(); // Disable solver controls for now
    ImGui::SliderInt("Max Solver Steps", &gui_config.max_solver_steps, 1, 200);
    ImGui::Checkbox("Show Solver Steps", &gui_config.show_solver_steps);

    if (ImGui::Button("Run Solver")) {
        runSolver();
    }
    //ImGui::EndDisabled();

    ImGui::Separator();
    ImGui::Text("Statistics:");
    ImGui::Text("Rectangles: %zu", display_placements.size());
    ImGui::Text("Boxes Used: %d", num_boxes);

    // Calculate utilization for current view
    if (num_boxes > 0) {
        float total_area;
        float used_area = 0;

        if (gui_config.view_all_boxes) {
            total_area = num_boxes * box_length * box_length;
            for (const auto& placement : display_placements) {
                used_area += placement.width * placement.height;
            }
        } else {
            total_area = box_length * box_length;
            for (const auto& placement : display_placements) {
                if (placement.box_id == gui_config.current_box_view) {
                    used_area += placement.width * placement.height;
                }
            }
        }

        if (total_area > 0) {
            float utilization = (used_area / total_area) * 100.0f;
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