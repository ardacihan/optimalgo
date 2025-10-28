#ifndef INSTANCEGENERATOR_H
#define INSTANCEGENERATOR_H

#include "RectanglePlacement.h"
#include <vector>
#include <random>

class InstanceGenerator {
private:
    int L;
    int min_width;
    int max_width;
    int min_height;
    int max_height;

public:
    InstanceGenerator(int L, int min_width, int max_width, int min_height, int max_height);
    std::vector<RectanglePlacement> generate_rectangles(int num_rectangles);
};

#endif // INSTANCEGENERATOR_H