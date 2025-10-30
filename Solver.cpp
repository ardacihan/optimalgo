//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include <vector>
#include <algorithm>
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
    auto bounding_boxes = problem.group_rectangles_by_bounding_box(solution);
    int L = problem.get_box_length();

    // Strategy 1: Try to place rectangles in empty spaces below existing rectangles
    for (auto& rect : solution) {
        // For each rectangle, try to find empty space below it in the same box
        auto rectangles_in_same_box = problem.getPlacementsInSameBoundingBoxRef(rect.box_id);

        // Find the bottom-most position in the current column
        int current_bottom = rect.y + rect.get_actual_height();

        // Try to place below if there's space
        if (current_bottom + rect.get_actual_height() <= L) {
            RectanglePlacement below_placement = rect;
            below_placement.y = current_bottom;

            // Check if this position is valid (no collisions)
            bool valid = true;
            for (const auto& existing : rectangles_in_same_box) {
                if (below_placement.collides(existing)) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                auto neighbor_solution = solution;
                for (auto& r : neighbor_solution) {
                    if (r.equals(rect)) {
                        r = below_placement;
                        break;
                    }
                }
                neighborhood.push_back(neighbor_solution);
            }
        }

        // Strategy 2: Try to place to the right of existing rectangles
        int current_right = rect.x + rect.get_actual_width();
        if (current_right + rect.get_actual_width() <= L) {
            RectanglePlacement right_placement = rect;
            right_placement.x = current_right;

            bool valid = true;
            for (const auto& existing : rectangles_in_same_box) {
                if (right_placement.collides(existing)) {
                    valid = false;
                    break;
                }
            }

            if (valid) {
                auto neighbor_solution = solution;
                for (auto& r : neighbor_solution) {
                    if (r.equals(rect)) {
                        r = right_placement;
                        break;
                    }
                }
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
    return solve(problem, max_steps - 1);
}

