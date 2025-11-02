#pragma once
#include <vector>

struct Rectangle {
    int x, y;
    int width, height;
    int box_id;
    bool rotated;
};

class RectangleVisualizer {
private:
    void* window; // GLFWwindow* hidden to avoid includes
    std::vector<Rectangle> rectangles;
    int box_length;
    float scale_factor;
    struct Offset { float x, y; } offset;
    bool initialized;

public:
    RectangleVisualizer(int width = 1200, int height = 800);
    ~RectangleVisualizer();

    bool initialize();
    void render();
    bool shouldClose() const;
    void pollEvents();

    void setRectangles(const std::vector<Rectangle>& rects);
    void setBoxLength(int length);

private:
    void updateScaleAndOffset();
};