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
    int move_id = -1;

    // Default constructor
    RectanglePlacement() : width(0), height(0), x(0), y(0), rotated(false), box_id(0) {}

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

    bool eq(RectanglePlacement& other) {

        return  (other.box_id == box_id && other.x == x && other.y == y);
    }

    bool adjacent(const RectanglePlacement& other) const {
        if (other.box_id != box_id) return false;

        int w1 = get_actual_width();
        int h1 = get_actual_height();
        int w2 = other.get_actual_width();
        int h2 = other.get_actual_height();


        int left1 = x;
        int right1 = x + w1;
        int bottom1 = y;
        int top1 = y + h1;

        int left2 = other.x;
        int right2 = other.x + w2;
        int bottom2 = other.y;
        int top2 = other.y + h2;


        bool right_edge_touching = (right1 == left2) && !(top1 <= bottom2 || bottom1 >= top2);

        bool left_edge_touching = (left1 == right2) && !(top1 <= bottom2 || bottom1 >= top2);

        bool top_edge_touching = (top1 == bottom2) && !(right1 <= left2 || left1 >= right2);

        bool bottom_edge_touching = (bottom1 == top2) && !(right1 <= left2 || left1 >= right2);

        return right_edge_touching || left_edge_touching || top_edge_touching || bottom_edge_touching;

    }
};

#endif // RECTANGLEPLACEMENT_H
