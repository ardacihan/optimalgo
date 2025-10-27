#ifndef RECTANGLE_H
#define RECTANGLE_H

class Rectangle {
public:
    const int width;
    const int height;
    const bool rotated;

    Rectangle(int w, int h, bool rot);
    int area() const;
};

#endif