//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include "OptimizationProblem.h"
#include <vector>

std::vector<std::unique_ptr<Input>> GeometryBasedNeighborhoodSolver::construct_neighbors(OptimizationProblem& problem, const Input& input) {
    std::vector<std::unique_ptr<Input>> neighborhood;
    auto neighbor = 0;
    // get each rectangle
    // move them around
    // move them from a box to another box
    // check constraints


    // reward if total sum of boxes are better -> this is by choosing the neighbors
    // also reward them if moving them closer to other  -> this is by choosing the neighbors


    return neighborhood;
}