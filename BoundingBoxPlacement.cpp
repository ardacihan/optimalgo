#include "BoundingBoxPlacement.h"

bool BoundingBoxPlacement::checkOverlapWithOtherRectangles(const RectanglePlacement &new_r) {
    for (RectanglePlacement r : placements) {
        if (r.rectangle.rotated) {
            if (r.x + r.rectangle.height <= new_r.x ||
                new_r.x + new_r.rectangle.width <= r.x ||
                r.y + r.rectangle.width <= new_r.y ||
                new_r.y + new_r.rectangle.height <= r.y) {
                return false;
                }
        } else {
            if (r.x + r.rectangle.width <= new_r.x ||
                new_r.x + new_r.rectangle.width <= r.x ||
                r.y + r.rectangle.height <= new_r.y ||
                new_r.y + new_r.rectangle.height <= r.y) {
                return false;
                }
        }
    }
    return true;
}

bool BoundingBoxPlacement::checkPlacementsWithinBoundingBox(const RectanglePlacement &new_r) {
    Rectangle rectangle = new_r.rectangle;
    bool a, b;
    if (rectangle.rotated) {
        a = new_r.x + rectangle.height < L;
        b = new_r.y + rectangle.width < L;
    } else {
        a = new_r.x + rectangle.width < L;
        b = new_r.y + rectangle.height < L;
    }
    if (!a || !b) {return false;}

    return true;
}

bool BoundingBoxPlacement::place_rectangle(const RectanglePlacement &new_r) {
    if (checkOverlapWithOtherRectangles(new_r) && checkPlacementsWithinBoundingBox(new_r)) {
        placements.push_back(new_r);
        return true;
    }
    return false;
}