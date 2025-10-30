//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include "OptimizationProblem.h"
#include <vector>
#include <memory>
#include <set>
#include "RectangleFittingProblem.h"
#include "RectanglePlacement.h"
#include <iostream>
#include <map>
#include <numeric>

// get each rectangle
// move them around
// move them from a box to another box
// check constraints


// reward if total sum of boxes are better -> this is by choosing the neighbors
// also reward them if moving them closer to other  -> this is by choosing the neighbors
#include <algorithm>
#include <cmath>
#include <iostream>

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {
    std::cout << "[Neighborhood] Constructing efficient neighbors..." << std::endl;

    std::vector<std::vector<RectanglePlacement>> neighborhood;
    const auto &solution = problem.get_current_solution();
    auto bounding_boxes = problem.group_rectangles_by_bounding_box(solution);
    int L = problem.get_box_length();

    const int EDGE_STEPS = 3; // sliding steps along edges

    for (size_t from_idx = 0; from_idx < bounding_boxes.size(); ++from_idx) {
        for (const auto &rect : bounding_boxes[from_idx]) {

            for (size_t to_idx = 0; to_idx < bounding_boxes.size(); ++to_idx) {
                if (to_idx == from_idx) continue;

                const auto &target_ref = bounding_boxes[to_idx].front();
                int target_box_id = target_ref.box_id;

                // Edge-based candidate positions (corners + edges)
                std::vector<std::pair<int, int>> positions = {
                    {0, 0}, {L - rect.width, 0}, {0, L - rect.height}, {L - rect.width, L - rect.height}
                };

                for (int s = 1; s < EDGE_STEPS; ++s) {
                    int step_x = s * (L - rect.width) / EDGE_STEPS;
                    int step_y = s * (L - rect.height) / EDGE_STEPS;
                    positions.push_back({step_x, 0});                 // top
                    positions.push_back({step_x, L - rect.height});   // bottom
                    positions.push_back({0, step_y});                 // left
                    positions.push_back({L - rect.width, step_y});    // right
                }

                for (auto [nx, ny] : positions) {
                    for (int rot = 0; rot < 2; ++rot) {
                        int w = rot ? rect.height : rect.width;
                        int h = rot ? rect.width  : rect.height;

                        //EARLY CULLING: skip placements outside box boundaries
                        if (nx < 0 || ny < 0 || nx + w > L || ny + h > L)
                            continue;

                        // Copy and modify
                        auto neighbor = solution;
                        for (auto &r : neighbor) {
                            if (r.box_id == rect.box_id && r.x == rect.x && r.y == rect.y) {
                                r.x = nx;
                                r.y = ny;
                                r.box_id = target_box_id;
                                r.rotated = (rot != 0);
                                break;
                            }
                        }

                        if (problem.check_no_overlaps(neighbor))
                            neighborhood.push_back(std::move(neighbor));
                    }
                }
            }
        }
    }

    return neighborhood;
}


std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::select_next_solution(RectangleFittingProblem &problem,
    std::vector<std::vector<RectanglePlacement>> neighborhood) {
    std::vector<RectanglePlacement> next_solution = neighborhood[0]; //select first one as a placeholder
    int next_solution_objective = problem.objective(next_solution);
    std::cout << "=== CALCULATING NEIGHBOR OBJECTIVE SCORES ===" << std::endl;

    for (std::vector<RectanglePlacement> solution : neighborhood) {
        int obj =  problem.objective(solution);
        std::cout << "Solution with obj score;" << obj << std::endl;

        if (obj >= next_solution_objective) {
            next_solution = solution;
            next_solution_objective = obj;
        }
    }









    return next_solution;
}


std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem& problem,int max_steps) {
    std::cout << "=== START SOLVE WITH REMAINING " << max_steps << " STEPS" << std::endl;

    std::vector<RectanglePlacement> initial_solution = problem.get_current_solution();
    if (max_steps <= 0) {
        std::cout << "=== END SOLVE"  << std::endl;
        return initial_solution;
    }
    std::vector<RectanglePlacement> next_solution =
        select_next_solution( problem, construct_neighbors(problem));
    problem.set_current_solution(next_solution);
    return solve(problem,max_steps - 1);
}


