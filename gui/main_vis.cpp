#include "visualizer.h"

int main() {
    RectangleVisualizer visualizer(1200, 800);
    if (!visualizer.initialize()) {
        return -1;
    }

    // Main loop
        while (!visualizer.shouldClose()) {
        visualizer.pollEvents();
        visualizer.render();
    }

    return 0;
}