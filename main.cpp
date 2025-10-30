#include "RectangleFittingProblem.h"
#include "InstanceGenerator.h"
#include "Solver.h"
#include <iostream>

// Simple test implementation


int main() {
    int L = 15;
    int num_rectangles = 20; // Smaller for testing
    int max_steps = 100;
    InstanceGenerator instance_generator(L, 1, 10, 1, 10);
    std::vector<RectanglePlacement> rectangles = instance_generator.generate_rectangles(num_rectangles);

    RectangleFittingProblem problem(L, rectangles);

    std::cout << "Initial solution set with " << rectangles.size() << " rectangles" << std::endl;

    GeometryBasedNeighborhoodSolver solver;
    std::vector<RectanglePlacement> solution = solver.solve(problem, max_steps);

    // Get coverage using the class method
    std::unordered_map<int, int> coverage_per_box = problem.get_coverage_each_bounding_box();

    std::cout << "Solution coverage per box:" << std::endl;
    for (const auto& [box_id, coverage] : coverage_per_box) {
        int percentage = (coverage * 100) / (L * L);
        std::cout << "Box " << box_id << ": " << coverage << "/" << (L * L)
                  << " (" << percentage << "%)" << std::endl;
    }
    std::cout << "Total boxes used: " << coverage_per_box.size() << std::endl;
    return 0;
}