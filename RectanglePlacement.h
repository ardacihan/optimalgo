#ifndef RECTANGLEPLACEMENT_H
#define RECTANGLEPLACEMENT_H

#include "Input.h"
#include <vector>
#include <memory>

struct Point {
    int x, y;
    Point(int x = 0, int y = 0) : x(x), y(y) {}
};

class RectanglePlacement : public Input {
public:
    int width;
    int height;
    int x;
    int y;
    bool rotated;
    int box_id;

    // Constructor
    RectanglePlacement(int w = 0, int h = 0, int x_pos = 0, int y_pos = 0,
                      bool rot = false, int bid = -1)
        : width(w), height(h), x(x_pos), y(y_pos), rotated(rot), box_id(bid) {}

    // Get actual width considering rotation
    int get_actual_width() const {
        return rotated ? height : width;
    }

    // Get actual height considering rotation
    int get_actual_height() const {
        return rotated ? width : height;
    }

    void rotate() { this->rotated = !this->rotated; }

};

#endif // RECTANGLEPLACEMENT_H