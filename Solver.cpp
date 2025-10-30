//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>

// get each rectangle
// move them around
// move them from a box to another box
// check constraints


// reward if total sum of boxes are better -> this is by choosing the neighbors
// also reward them if moving them closer to other  -> this is by choosing the neighbors



std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {
    std::vector<std::vector<RectanglePlacement>> neighborhood;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();

    // Get rectangles grouped by bounding box
    auto box_groups = problem.group_rectangles_by_bounding_box(solution);

    // For each rectangle, try moving it to each other bounding box
    for (size_t rect_idx = 0; rect_idx < solution.size(); rect_idx++) {
        auto& rect = solution[rect_idx];
        int current_box = rect.box_id;

        for (const auto& target_group : box_groups) {
            if (target_group.empty()) continue;

            int target_box = target_group[0].box_id;
            if (target_box == current_box) continue;

            // Get all rectangles in target box
            auto target_rects = problem.getPlacementsInSameBoundingBoxRef(target_box);

            // Find bottom-left position for this rectangle in target box
            bool found_position = false;
            RectanglePlacement new_placement = rect;
            new_placement.box_id = target_box;

            // Simple BL: Try along bottom, then move up row by row
            for (int y = 0; y <= L - new_placement.get_actual_height() && !found_position; y++) {
                for (int x = 0; x <= L - new_placement.get_actual_width() && !found_position; x++) {
                    new_placement.x = x;
                    new_placement.y = y;

                    // Check if position is valid
                    bool valid = true;
                    for (const auto& existing : target_rects) {
                        if (new_placement.collides(existing)) {
                            valid = false;
                            break;
                        }
                    }

                    // Check within box boundaries
                    if (valid &&
                        new_placement.x + new_placement.get_actual_width() <= L &&
                        new_placement.y + new_placement.get_actual_height() <= L) {
                        found_position = true;
                        break;
                    }
                }
            }

            // If we found a valid position, create neighbor solution
            if (found_position) {
                auto neighbor_solution = solution;
                neighbor_solution[rect_idx] = new_placement;
                neighborhood.push_back(neighbor_solution);
            }
        }
    }

    return neighborhood;
}

std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();
    if (max_steps <= 0) return solution;

    auto neighbors = construct_neighbors(problem);
    if (neighbors.empty()) return solution;

    // Select neighbor with best objective
    std::vector<RectanglePlacement> next_solution = neighbors[0];
    int best_obj = problem.objective(next_solution);
    for (auto &n : neighbors) {
        int obj = problem.objective(n);
        if (obj > best_obj) {
            best_obj = obj;
            next_solution = n;
        }
    }

    problem.set_current_solution(next_solution);
    std::cout <<"Taking the next step with obj score: " <<best_obj << std::endl;
    return solve(problem, max_steps - 1);
}

