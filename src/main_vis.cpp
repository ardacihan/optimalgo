#include "visualizer.h"
#include <thread>
#include <chrono>
#include <iostream>

int main() {
    std::cout << "Starting Rectangle Packing Visualizer..." << std::endl;

    RectangleVisualizer visualizer(1400, 900);

    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualizer!" << std::endl;
        return -1;
    }

    // Generate some initial rectangles
    std::vector<Rectangle> initial_rects = {
        {10, 10, 20, 15, 0, false},
        {40, 10, 15, 25, 0, false},
        {10, 40, 25, 20, 1, false},
        {50, 50, 30, 10, 1, false},
        {15, 15, 20, 20, 2, false},
        {60, 20, 15, 15, 2, false}
    };

    visualizer.setRectangles(initial_rects);
    visualizer.setBoxLength(100);

    std::cout << "Entering main loop..." << std::endl;

    // Main loop
    while (!visualizer.shouldClose()) {
        visualizer.pollEvents();
        visualizer.render();
        std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60 FPS
    }

    std::cout << "Visualizer closed successfully." << std::endl;
    return 0;
}