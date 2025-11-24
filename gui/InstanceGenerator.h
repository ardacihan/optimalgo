#ifndef INSTANCEGENERATOR_H
#define INSTANCEGENERATOR_H

#include "../problem/RectanglePlacement.h"
#include <vector>

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
    
    std::vector<RectanglePlacement> create_better_initial_solution(
        std::vector<RectanglePlacement>& rectangles, int box_length);
};

#endif