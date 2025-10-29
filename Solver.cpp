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


// get each rectangle
// move them around
// move them from a box to another box
// check constraints


// reward if total sum of boxes are better -> this is by choosing the neighbors
// also reward them if moving them closer to other  -> this is by choosing the neighbors
std::vector<std::vector<RectanglePlacement>> GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {
    std::vector<std::vector<RectanglePlacement>> neighborhood;
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    // Translation neighborhood
    std::cout << "=== GENERATING TRANSLATION NEIGHBORS ===" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i) {
        for (int dx : {-2, -1, 0, 1, 2}) { //move right left a bit
            for (int dy : {-2, -1, 0, 1, 2}) { //move up down a bit
                auto neighbor = solution;
                if (neighbor[i].x + dx >= 0 && neighbor[i].y + dy >= 0) {
                    neighbor[i].x += dx;
                    neighbor[i].y += dy;
                    if (problem.check_no_overlaps(neighbor) && problem.check_within_boxes(neighbor)) {
                        std::cout << "Moving rect " << i << " (" << solution[i].width << "x" << solution[i].height
                                  << ") in box " << solution[i].box_id << " from (" << solution[i].x << "," << solution[i].y
                                  << ") to (" << neighbor[i].x << "," << neighbor[i].y << ")" << std::endl;
                        neighborhood.push_back(neighbor); //check legal
                    }
                }
            }
        }
    }

    // Rotation neighborhood
    std::cout << "=== GENERATING ROTATION NEIGHBORS ===" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i) {
        auto rotated_neighbor = solution;
        rotated_neighbor[i].rotate();
        if (problem.check_no_overlaps(rotated_neighbor) && problem.check_within_boxes(rotated_neighbor)) {
            std::cout << "Rotating rect " << i << " from " << solution[i].width << "x" << solution[i].height
                      << " to " << rotated_neighbor[i].width << "x" << rotated_neighbor[i].height
                      << " in box " << solution[i].box_id << " at position (" << solution[i].x << "," << solution[i].y << ")" << std::endl;
            neighborhood.push_back(rotated_neighbor); //check legal after flipping stuff
            //TODO how do we know if flipping is good after some steps but indifferent initially?
        }
    }

     std::cout << "=== GENERATING SMART BOX-SWAP NEIGHBORS ===" << std::endl;

    // Count rectangles per box
    std::map<int, int> box_counts;
    for (const auto& rect : solution) {
        box_counts[rect.box_id]++;
    }

    // Identify almost-empty boxes (less than 1/4 of total rectangles)
    double threshold = solution.size() / 4.0;
    std::vector<int> almost_empty_boxes;
    for (const auto& [box_id, count] : box_counts) {
        if (count < threshold) {
            almost_empty_boxes.push_back(box_id);
            std::cout << "Box " << box_id << " is almost-empty with " << count << " rectangles" << std::endl;
        }
    }

    // If no almost-empty boxes, use the box with minimum rectangles
    if (almost_empty_boxes.empty()) {
        auto min_box = std::min_element(box_counts.begin(), box_counts.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
        almost_empty_boxes.push_back(min_box->first);
        std::cout << "No almost-empty boxes. Using box " << min_box->first
                  << " with " << min_box->second << " rectangles" << std::endl;
    }

    // For each rectangle in almost-empty boxes, try to move it to other boxes
    for (size_t i = 0; i < solution.size(); ++i) {
        int current_box = solution[i].box_id;

        // Only consider rectangles in almost-empty boxes
        if (std::find(almost_empty_boxes.begin(), almost_empty_boxes.end(), current_box) == almost_empty_boxes.end()) {
            continue;
        }

        // Try moving to all other boxes
        for (const auto& [target_box, count] : box_counts) {
            if (target_box == current_box) continue;

            std::cout << "Trying to move rect " << i << " (" << solution[i].width << "x" << solution[i].height
                      << ") from almost-empty box " << current_box << " to box " << target_box << std::endl;

            // Try multiple positions in target box instead of just (0,0)
            std::vector<std::pair<double, double>> candidate_positions = {
                {0, 0},  // Bottom-left
                {0, 5},  // Slightly up
                {5, 0},  // Slightly right
                {5, 5}   // Diagonal
            };

            // Also try positions near existing rectangles in target box
            for (const auto& rect : solution) {
                if (rect.box_id == target_box) {
                    candidate_positions.push_back({rect.x, rect.y + rect.get_actual_height() + 1});
                    candidate_positions.push_back({rect.x + rect.get_actual_width() + 1, rect.y});
                }
            }

            // Try each candidate position
            for (const auto& [x, y] : candidate_positions) {
                auto neighbor = solution;
                neighbor[i].box_id = target_box;
                neighbor[i].x = x;
                neighbor[i].y = y;

                if (problem.check_no_overlaps(neighbor) && problem.check_within_boxes(neighbor)) {
                    std::cout << "SUCCESS: Moved rect " << i << " from box " << current_box
                              << " to box " << target_box << " at position (" << x << "," << y << ")" << std::endl;
                    neighborhood.push_back(neighbor);
                    break; // Found a valid position, move to next target box
                }
            }
        }
    }

    std::cout << "=== TOTAL NEIGHBORS GENERATED: " << neighborhood.size() << " ===" << std::endl;
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


std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem& problem) {
    std::vector<RectanglePlacement> initial_solution = problem.get_current_solution();


}


