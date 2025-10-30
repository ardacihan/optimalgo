#ifndef RECTANGLEPLACEMENT_H
#define RECTANGLEPLACEMENT_H

#include "Input.h"
#include <vector>
#include <memory>


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

    bool equals(RectanglePlacement h) {
        return h.height == this->height && h.width == this->width &&  h.rotated == rotated &&  h.box_id == box_id;
    }

    bool collides(const RectanglePlacement& other) const {
        // Get the actual dimensions considering rotation
        int this_width = get_actual_width();
        int this_height = get_actual_height();
        int other_width = other.get_actual_width();
        int other_height = other.get_actual_height();

        // Check if rectangles don't overlap (no collision)
        if (x + this_width <= other.x ||  // This is to the left of other
            other.x + other_width <= x ||  // Other is to the left of this
            y + this_height <= other.y ||  // This is above other
            other.y + other_height <= y) { // Other is above this
            return false; // No collision
            }

        return true; // Collision detected
    }

    int getOverlapArea(const RectanglePlacement& other) const {
        if (!collides(other)) {
            return 0;
        }

        int this_width = get_actual_width();
        int this_height = get_actual_height();
        int other_width = other.get_actual_width();
        int other_height = other.get_actual_height();

        // Calculate overlap rectangle
        int overlap_left = std::max(x, other.x);
        int overlap_right = std::min(x + this_width, other.x + other_width);
        int overlap_top = std::max(y, other.y);
        int overlap_bottom = std::min(y + this_height, other.y + other_height);

        int overlap_width = overlap_right - overlap_left;
        int overlap_height = overlap_bottom - overlap_top;

        return overlap_width * overlap_height;
    }
};

#endif // RECTANGLEPLACEMENT_H