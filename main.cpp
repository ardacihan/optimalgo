#include "RectangleFittingProblem.h"
#include "InstanceGenerator.h"
#include "Solver.h"
#include <iostream>

// Simple test implementation


int main() {
    int L = 15;
    int num_rectangles = 4; // Smaller for testing
    InstanceGenerator instance_generator(L, 1, 6, 1, 5);
    std::vector<RectanglePlacement> rectangles = instance_generator.generate_rectangles(num_rectangles);

    RectangleFittingProblem problem(L, rectangles);

    std::cout << "Initial solution set with " << rectangles.size() << " rectangles" << std::endl;

    GeometryBasedNeighborhoodSolver solver;
    auto result = solver.construct_neighbors(problem);
    solver.select_next_solution(problem,result);


    std::cout << "Test completed successfully" << std::endl;
    return 0;
}