//
// RectanglePlacement.h
//

#ifndef RECTANGLEPLACEMENT_H
#define RECTANGLEPLACEMENT_H

#include <algorithm>

struct RectanglePlacement {
    int width;
    int height;
    int x;
    int y;
    bool rotated;
    int box_id;

    RectanglePlacement(int w, int h, int x_pos, int y_pos, bool rot, int box)
        : width(w), height(h), x(x_pos), y(y_pos), rotated(rot), box_id(box) {}

    int get_actual_width() const {
        return rotated ? height : width;
    }

    int get_actual_height() const {
        return rotated ? width : height;
    }

    bool collides(const RectanglePlacement& other) const {
        int w1 = get_actual_width();
        int h1 = get_actual_height();
        int w2 = other.get_actual_width();
        int h2 = other.get_actual_height();

        return !(x + w1 <= other.x || other.x + w2 <= x ||
                 y + h1 <= other.y || other.y + h2 <= y);
    }

    long long getOverlapArea(const RectanglePlacement& other) const {
        int w1 = get_actual_width();
        int h1 = get_actual_height();
        int w2 = other.get_actual_width();
        int h2 = other.get_actual_height();

        int x_overlap = std::max(0, std::min(x + w1, other.x + w2) - std::max(x, other.x));
        int y_overlap = std::max(0, std::min(y + h1, other.y + h2) - std::max(y, other.y));

        return (long long)x_overlap * y_overlap;
    }
};

#endif // RECTANGLEPLACEMENT_H
