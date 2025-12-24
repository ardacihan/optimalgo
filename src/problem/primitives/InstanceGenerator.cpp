//
// InstanceGenerator.cpp
//

#include "InstanceGenerator.h"
#include <random>
#include <algorithm>
#include <iostream>

InstanceGenerator::InstanceGenerator(int L, int min_width, int max_width, int min_height, int max_height)
    : L(L), min_width(min_width), max_width(max_width), min_height(min_height), max_height(max_height) {}

std::vector<RectanglePlacement> InstanceGenerator::generate_rectangles(int num_rectangles) {
    std::vector<RectanglePlacement> rectangle_placements;
    rectangle_placements.reserve(num_rectangles);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> height_dist(min_height, max_height);
    std::uniform_int_distribution<> width_dist(min_width, max_width);

    for (int i = 0; i < num_rectangles; i++) {
        int height = height_dist(gen);
        int width = width_dist(gen);
        rectangle_placements.emplace_back(width, height, 0, 0, false, i);
    }

    return rectangle_placements;
}
