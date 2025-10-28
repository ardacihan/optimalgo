#include <iostream>
#include <random>
#include <vector>
#include "InstanceGenerator.h"
#include "OptimizationProblem.h"
#include "RectangleFittingProblem.h"
#include "RectanglePlacement.h"



int main() {

    int L = 15;
    int num_rectangles = 10;
    InstanceGenerator instance_generator = InstanceGenerator(L,1,9,1,13);
    std::vector<RectanglePlacement> rectangles = instance_generator.generate_rectangles(10);




    return 0;
}