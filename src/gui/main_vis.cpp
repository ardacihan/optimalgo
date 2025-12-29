#include "visualizer.h"

int main() {
    RectangleVisualizer visualizer(1920, 1080);
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