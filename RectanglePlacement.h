#ifndef RECTANGLE_PLACEMENT_H
#define RECTANGLE_PLACEMENT_H

#include "Rectangle.h"

class RectanglePlacement {
public:
    Rectangle rectangle;
    int x, y;
    int boundingBoxId;

    RectanglePlacement(Rectangle r, int x, int y, int boundingBoxId);
};

#endif