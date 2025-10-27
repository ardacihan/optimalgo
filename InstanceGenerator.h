#ifndef INSTANCE_GENERATOR_H
#define INSTANCE_GENERATOR_H

#include <vector>
#include <random>
#include "Rectangle.h"

class InstanceGenerator {
private:
    int L;
    int min_width;
    int min_height;
    int max_width;
    int max_height;

public:
    InstanceGenerator(int L, int min_width, int max_width, int min_height, int max_height);
    std::vector<Rectangle> generate_rectangles(int num_rectangles);
};

#endif