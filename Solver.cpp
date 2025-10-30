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

    std::stack<RectanglePlacement> rectangle_candidates;

    for (auto rectangles : solution) {
        int size = rectangles.height * rectangles.width;

        for (auto searched_rectangles : solution) { //for each rectangle, find a bigger rectangle to attach to
            if (!searched_rectangles.equals(rectangles)) { // another rectangle needed for attachment
                if (searched_rectangles.box_id != rectangles.box_id) { //we want to attach to another box
                    if (searched_rectangles.height * searched_rectangles.width > size) {
                        rectangle_candidates.push(searched_rectangles);
                    }
                }
            }
        }

        while (!rectangle_candidates.empty()) {
            auto r = rectangle_candidates.top();

            // Try to place adjacent to the right first
            RectanglePlacement right_placement = rectangles;
            right_placement.box_id = r.box_id;
            right_placement.x = r.x + r.get_actual_width();
            right_placement.y = r.y;

            // Try to place on top if right placement doesn't fit
            RectanglePlacement top_placement = rectangles;
            top_placement.box_id = r.box_id;
            top_placement.x = r.x;
            top_placement.y = r.y + r.get_actual_height();

            auto rectangles_in_the_same_box = problem.getPlacementsInSameBoundingBoxRef(r.box_id);

            // Check if right placement is valid
            bool right_valid = (right_placement.x + right_placement.get_actual_width() <= L) &&
                              (right_placement.y + right_placement.get_actual_height() <= L);

            // Check if top placement is valid
            bool top_valid = (top_placement.x + top_placement.get_actual_width() <= L) &&
                            (top_placement.y + top_placement.get_actual_height() <= L);

            // Check for collisions with other rectangles in the same bounding box
            if (right_valid) {
                bool has_collision = false;
                for (const auto& existing_rect : rectangles_in_the_same_box) {
                    if (right_placement.collides(existing_rect)) {
                        has_collision = true;
                        break;
                    }
                }
                if (!has_collision) {
                    // Create neighbor solution with right placement
                    std::vector<RectanglePlacement> neighbor_solution = solution;
                    // Replace the original rectangle with the new placement
                    for (auto& rect : neighbor_solution) {
                        if (rect.equals(rectangles)) {
                            rect = right_placement;
                            break;
                        }
                    }
                    neighborhood.push_back(neighbor_solution);
                }
            }

            // If right placement didn't work, try top placement
            if (!right_valid || neighborhood.empty()) {
                if (top_valid) {
                    bool has_collision = false;
                    for (const auto& existing_rect : rectangles_in_the_same_box) {
                        if (top_placement.collides(existing_rect)) {
                            has_collision = true;
                            break;
                        }
                    }
                    if (!has_collision) {
                        // Create neighbor solution with top placement
                        std::vector<RectanglePlacement> neighbor_solution = solution;
                        // Replace the original rectangle with the new placement
                        for (auto& rect : neighbor_solution) {
                            if (rect.equals(rectangles)) {
                                rect = top_placement;
                                break;
                            }
                        }
                        neighborhood.push_back(neighbor_solution);
                    }
                }
            }

            rectangle_candidates.pop();
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

