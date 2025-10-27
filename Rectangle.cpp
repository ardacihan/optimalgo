#include "Rectangle.h"

Rectangle::Rectangle(int w, int h, bool rot) : width(w), height(h), rotated(rot) {}

int Rectangle::area() const { return width * height; }