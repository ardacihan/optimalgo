#ifndef RECTANGLEPLACEMENT_H
#define RECTANGLEPLACEMENT_H
#include "Input.h"

class RectanglePlacement : Input {
public:
    int width;
    int height;
    int x;
    int y;
    bool rotated;
    int box_id;

    // Constructor
    RectanglePlacement(int w, int h, int x_pos, int y_pos, bool rot, int bid)
        : width(w), height(h), x(x_pos), y(y_pos), rotated(rot), box_id(bid) {}

    // Default constructor
    RectanglePlacement() : width(0), height(0), x(0), y(0), rotated(false), box_id(-1) {}

    // Get actual width considering rotation
    int get_actual_width() const {
        return rotated ? height : width;
    }

    // Get actual height considering rotation
    int get_actual_height() const {
        return rotated ? width : height;
    }

    void rotate() { this->rotated = !this->rotated;}
};

#endif // RECTANGLEPLACEMENT_H