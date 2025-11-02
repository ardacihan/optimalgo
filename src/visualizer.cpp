#include "visualizer.h"

// Include OpenGL headers before GLFW
#include <GL/gl.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <iostream>
#include <random>
#include <algorithm>
#include <string>

RectangleVisualizer::RectangleVisualizer(int width, int height)
    : box_length(100), scale_factor(1.0f), offset{50.0f, 50.0f}, initialized(false) {

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return;
    }

    // Set GLFW window hints
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

    glfwMakeContextCurrent(static_cast<GLFWwindow*>(window));
    glfwSwapInterval(1); // VSync
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

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    if (!ImGui_ImplGlfw_InitForOpenGL(static_cast<GLFWwindow*>(window), true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend" << std::endl;
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        std::cerr << "Failed to initialize ImGui OpenGL backend" << std::endl;
        return false;
    }

    initialized = true;
    std::cout << "Visualizer initialized successfully" << std::endl;
    return true;
}

void RectangleVisualizer::setRectangles(const std::vector<Rectangle>& rects) {
    rectangles = rects;
    updateScaleAndOffset();
}

void RectangleVisualizer::setBoxLength(int length) {
    box_length = length;
    updateScaleAndOffset();
}

void RectangleVisualizer::updateScaleAndOffset() {
    // Simple auto-scaling
    float max_coord = box_length * 1.2f; // Some padding
    scale_factor = std::min(600.0f / max_coord, 10.0f); // Don't scale too much
    offset = {100.0f, 150.0f}; // More vertical offset for GUI
}

void RectangleVisualizer::pollEvents() {
    glfwPollEvents();
}

void RectangleVisualizer::render() {
    if (!initialized || !window) return;

    // Start ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Clear screen - using OpenGL functions
    int display_w, display_h;
    glfwGetFramebufferSize(static_cast<GLFWwindow*>(window), &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Get background draw list for custom rendering
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    // Calculate number of boxes needed (simplified - you'll replace this)
    int num_boxes = 1;
    if (!rectangles.empty()) {
        int max_box_id = 0;
        for (const auto& rect : rectangles) {
            if (rect.box_id > max_box_id) max_box_id = rect.box_id;
        }
        num_boxes = max_box_id + 1;
    }
    num_boxes = std::max(1, num_boxes); // At least one box

    // Draw grid and boxes
    float box_spacing = 20.0f;
    float total_width = num_boxes * (box_length * scale_factor) + (num_boxes - 1) * box_spacing;
    float start_x = (display_w - total_width) / 2.0f; // Center boxes horizontally

    for (int i = 0; i < num_boxes; ++i) {
        float box_x = start_x + i * (box_length * scale_factor + box_spacing);
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
        std::string box_label = "Box " + std::to_string(i + 1);
        draw_list->AddText(
            ImVec2(box_x + 5, box_y + 5),
            IM_COL32(255, 255, 255, 255),
            box_label.c_str()
        );
    }

    // Draw rectangles
    ImU32 colors[] = {
        IM_COL32(65, 105, 225, 200),  // Royal Blue
        IM_COL32(220, 20, 60, 200),   // Crimson
        IM_COL32(50, 205, 50, 200),   // Lime Green
        IM_COL32(255, 140, 0, 200),   // Dark Orange
        IM_COL32(148, 0, 211, 200),   // Dark Violet
        IM_COL32(255, 215, 0, 200),   // Gold
        IM_COL32(0, 206, 209, 200),   // Dark Turquoise
        IM_COL32(255, 99, 71, 200)    // Tomato
    };

    for (const auto& rect : rectangles) {
        int box_index = rect.box_id;
        if (box_index >= num_boxes) box_index = 0; // Safety check

        float box_x = start_x + box_index * (box_length * scale_factor + box_spacing);

        float rect_x = box_x + rect.x * scale_factor;
        float rect_y = offset.y + rect.y * scale_factor;
        float rect_w = rect.width * scale_factor;
        float rect_h = rect.height * scale_factor;

        // Check if rectangle fits in box
        if (rect_x + rect_w > box_x + box_length * scale_factor) {
            rect_w = (box_x + box_length * scale_factor) - rect_x;
        }
        if (rect_y + rect_h > offset.y + box_length * scale_factor) {
            rect_h = (offset.y + box_length * scale_factor) - rect_y;
        }

        ImU32 color = colors[rect.box_id % 8];

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

        // Size label (only if rectangle is big enough)
        if (rect_w > 20 && rect_h > 15) {
            std::string size_label = std::to_string(rect.width) + "x" + std::to_string(rect.height);
            if (rect.rotated) {
                size_label += " R";
            }

            ImVec2 text_size = ImGui::CalcTextSize(size_label.c_str());
            float text_x = rect_x + rect_w / 2 - text_size.x / 2;
            float text_y = rect_y + rect_h / 2 - text_size.y / 2;

            // Draw text with background for better readability
            draw_list->AddRectFilled(
                ImVec2(text_x - 2, text_y - 1),
                ImVec2(text_x + text_size.x + 2, text_y + text_size.y + 1),
                IM_COL32(0, 0, 0, 128)
            );

            draw_list->AddText(
                ImVec2(text_x, text_y),
                IM_COL32(255, 255, 255, 255),
                size_label.c_str()
            );
        }
    }

    // GUI Window
    ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Rectangle Packing Controls");

    ImGui::Text("OptimalGo Visualizer");
    ImGui::Separator();

    // Statistics
    ImGui::Text("Statistics:");
    ImGui::Text("Rectangles: %zu", rectangles.size());
    ImGui::Text("Box Size: %d", box_length);
    ImGui::Text("Boxes Used: %d", num_boxes);

    // Calculate utilization (simplified)
    float total_area = num_boxes * box_length * box_length;
    float used_area = 0;
    for (const auto& rect : rectangles) {
        used_area += rect.width * rect.height;
    }
    float utilization = (used_area / total_area) * 100.0f;
    ImGui::Text("Utilization: %.1f%%", utilization);

    ImGui::Separator();

    // Configuration
    static int rect_count = 10;
    static int min_size = 5;
    static int max_size = 30;

    ImGui::Text("Configuration:");
    ImGui::SliderInt("Rectangle Count", &rect_count, 1, 50);
    ImGui::SliderInt("Min Size", &min_size, 1, 50);
    ImGui::SliderInt("Max Size", &max_size, min_size + 1, 100);
    ImGui::SliderInt("Box Size (L)", &box_length, 50, 200);

    ImGui::Separator();

    // Controls
    if (ImGui::Button("Generate Random Layout")) {
        std::vector<Rectangle> new_rects;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> size_dist(min_size, max_size);
        std::uniform_int_distribution<> box_dist(0, 2);

        for (int i = 0; i < rect_count; ++i) {
            int width = size_dist(gen);
            int height = size_dist(gen);

            // Ensure rectangles fit in boxes
            int max_x = std::max(0, box_length - width);
            int max_y = std::max(0, box_length - height);

            std::uniform_int_distribution<> pos_x_dist(0, max_x);
            std::uniform_int_distribution<> pos_y_dist(0, max_y);

            new_rects.push_back({
                pos_x_dist(gen),
                pos_y_dist(gen),
                width,
                height,
                box_dist(gen),
                false
            });
        }
        setRectangles(new_rects);
        setBoxLength(box_length);
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear All")) {
        rectangles.clear();
    }

    ImGui::Separator();
    ImGui::Text("Instructions:");
    ImGui::Text("• Use sliders to configure");
    ImGui::Text("• Click 'Generate' to create");
    ImGui::Text("• Different colors = different boxes");

    ImGui::End();
    
    // Render ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
    glfwSwapBuffers(static_cast<GLFWwindow*>(window));
}

bool RectangleVisualizer::shouldClose() const {
    return glfwWindowShouldClose(static_cast<GLFWwindow*>(window));
}