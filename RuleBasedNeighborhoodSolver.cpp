//
// RuleBasedNeighborhoodSolver.cpp
// Works on permutations: The ORDER of rectangles in the vector matters!
// Greedy placement rule places them in that order.
//

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>

// ============== Greedy Placement Rule ==============
// Takes rectangles IN ORDER and places them greedily

std::vector<RectanglePlacement> RuleBasedNeighborhoodSolver::apply_greedy_placement(
    const std::vector<RectanglePlacement>& rectangles_in_order,
    int L) {

    std::vector<RectanglePlacement> result;

    // Track occupancy for each box
    std::unordered_map<int, std::vector<int>> occupancy_grids;
    int next_box_id = 0;

    auto collides = [&](int x, int y, int w, int h, int box_id) {
        if (occupancy_grids.find(box_id) == occupancy_grids.end()) return false;

        const auto& grid = occupancy_grids[box_id];

        for (int py = y; py < y + h; py++) {
            for (int px = x; px < x + w; px++) {
                if (px >= L || py >= L || px < 0 || py < 0) return true;
                if (grid[py * L + px] != -1) return true;
            }
        }
        return false;
    };

    auto mark_occupied = [&](int x, int y, int w, int h, int box_id, int rect_idx) {
        if (occupancy_grids.find(box_id) == occupancy_grids.end()) {
            occupancy_grids[box_id] = std::vector<int>(L * L, -1);
        }

        auto& grid = occupancy_grids[box_id];
        for (int py = y; py < y + h; py++) {
            for (int px = x; px < x + w; px++) {
                grid[py * L + px] = rect_idx;
            }
        }
    };

    // GREEDY PLACEMENT: For each rectangle in order, find first valid spot
    for (size_t i = 0; i < rectangles_in_order.size(); i++) {
        const RectanglePlacement& rect = rectangles_in_order[i];

        // Initialize with default values - will be set properly below
        RectanglePlacement placement(rect.width, rect.height, 0, 0, false, -1);

        bool placed = false;

        // Try to place in existing boxes first (CRITICAL for minimizing boxes!)
        for (int box_id = 0; box_id < next_box_id && !placed; box_id++) {

            // Try both orientations
            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && rect.width == rect.height) continue; // Skip duplicate orientation

                int w = (rot == 0) ? rect.width : rect.height;
                int h = (rot == 0) ? rect.height : rect.width;

                if (w > L || h > L) continue;

                // Greedy heuristic: Try bottom-left positions first
                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides(x, y, w, h, box_id)) {
                            placement = RectanglePlacement(rect.width, rect.height, x, y, (rot == 1), box_id);
                            placed = true;
                        }
                    }
                }
            }
        }

        // If no existing box works, create new box
        if (!placed) {
            // Choose best orientation
            bool rotated = false;
            if (rect.width > L && rect.height <= L) {
                rotated = true;
            } else if (rect.height > L && rect.width <= L) {
                rotated = false;
            } else if (rect.width <= L && rect.height <= L) {
                rotated = false; // Both fit, use original
            } else {
                // Neither fits - use original (shouldn't happen if problem is valid)
                rotated = false;
            }

            int w = rotated ? rect.height : rect.width;
            int h = rotated ? rect.width : rect.height;

            // Make sure it fits in the new box
            if (w > L || h > L) {
                // Force it to fit by using the smaller dimension
                if (rect.width <= L && rect.height <= L) {
                    w = rect.width;
                    h = rect.height;
                    rotated = false;
                } else if (rect.height <= L) {
                    w = rect.height;
                    h = rect.width;
                    rotated = true;
                } else if (rect.width <= L) {
                    w = rect.width;
                    h = rect.height;
                    rotated = false;
                } else {
                    // Shouldn't happen - rectangle too large for box
                    w = std::min(rect.width, L);
                    h = std::min(rect.height, L);
                }
            }

            placement = RectanglePlacement(rect.width, rect.height, 0, 0, rotated, next_box_id++);
        }

        result.push_back(placement);
        mark_occupied(placement.x, placement.y,
                     placement.get_actual_width(), placement.get_actual_height(),
                     placement.box_id, i);
    }

    return result;
}

// ============== Neighborhood: Reorder the vector ==============

std::vector<std::vector<RectanglePlacement>> RuleBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> neighbors;
    auto solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 50;

    if (n == 0) return neighbors;

    // Extract rectangle dimensions (order matters)
    std::vector<RectanglePlacement> rect_order;
    for (const auto& placement : solution) {
        RectanglePlacement r(placement.width, placement.height, 0, 0, false, 0);
        rect_order.push_back(r);
    }

    // Swap each rectangle with every other rectangle
    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {
            auto reordered = rect_order;
            std::swap(reordered[i], reordered[j]);

            auto new_solution = apply_greedy_placement(reordered, L);
            neighbors.push_back(new_solution);
        }
    }

    std::cout << "Generated " << neighbors.size() << " neighbors" << std::endl;
    return neighbors;
}

// ============== Solver Implementation ==============

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps, int max_rectangle_in_subproblem) {
    return solve_with_reruns(problem,max_steps,max_rectangle_in_subproblem);
}

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::solve_one_step(RectangleFittingProblem &problem) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    auto neighbors = construct_neighbors(problem);
    if (neighbors.empty()) {
        std::cout << "No neighbors generated." << std::endl;
        return solution;
    }

    int current_obj = problem.objective(solution);

    std::vector<RectanglePlacement> best_neighbor = solution;
    int best_obj = current_obj;

    for (auto &n : neighbors) {
        int obj = problem.objective(n);
        if (obj > best_obj) {
            best_obj = obj;
            best_neighbor = n;
        }
    }

    problem.set_current_solution(best_neighbor);
    return best_neighbor;
}


std::vector<RectanglePlacement> RuleBasedNeighborhoodSolver::solve_with_reruns(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    if (num_reruns <= 0) return solution;

    std::unordered_set<int> box_ids;
    for (const auto& placement : solution) {
        box_ids.insert(placement.box_id);
    }

    // A. Subproblem size is small enough: Apply local search
    if (box_ids.size() < max_rectangle_in_subproblem) {
        auto neighbors = construct_neighbors(problem);
        if (neighbors.empty()) {
            std::cout << "No neighbors generated, stopping." << std::endl;
            return solution;
        }

        int current_obj = problem.objective(solution);
        std::vector<RectanglePlacement> best_neighbor = solution;
        int best_obj = current_obj;

        for (auto &n : neighbors) {
            int obj = problem.objective(n);
            if (obj > best_obj) {
                best_obj = obj;
                best_neighbor = n;
            }
        }

        if (best_obj > current_obj) {
            // Continue local search with the new, better solution
            problem.set_current_solution(best_neighbor);
            return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
        } else {
            // Local optimum reached, return the solution for merging/rerun logic in the caller
            return solution;
        }
    }
    // B. Subproblem is too large: Divide-and-Conquer
    else {
        // --- Divide ---
        auto [left, right] = splitRectanglesByBoxId(solution);
        int L = problem.get_box_length();

        RectangleFittingProblem p1(L, left);
        RectangleFittingProblem p2(L, right);

        // --- Conquer (Recursive Calls) ---
        // Pass the full num_reruns to the subproblems
        auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
        auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

        // --- Merge ---
        std::vector<RectanglePlacement> merged = solved_left;
        merged.insert(merged.end(), solved_right.begin(), solved_right.end());

        problem.set_current_solution(merged);

        // --- Rerun Logic (Filter and Re-optimize) ---
        if (num_reruns > 1) {
            auto coverage = problem.get_coverage_each_bounding_box();
            std::vector<RectanglePlacement> remaining_solution;
            int total_box_area = L * L;
            int threshold = total_box_area * 0.7;

            // Filter out rectangles in boxes that are NOT filled
            for (const auto& placement : merged) {
                if (coverage[placement.box_id] <= threshold) {
                    remaining_solution.push_back(placement);
                }
            }

            if (!remaining_solution.empty()) {
                std::unordered_set<int> remaining_box_ids;
                for (const auto& placement : remaining_solution) {
                    remaining_box_ids.insert(placement.box_id);
                }

                if (remaining_box_ids.size() >= max_rectangle_in_subproblem) {
                    RectangleFittingProblem remaining_problem(L, remaining_solution);

                    // Decrement num_reruns for the re-optimization step
                    auto optimized_remaining = solve_with_reruns(remaining_problem, num_reruns - 1, max_rectangle_in_subproblem);

                    std::vector<RectanglePlacement> final_solution;
                    // Keep the 'filled' ones
                    for (const auto& placement : merged) {
                        if (coverage[placement.box_id] > threshold) {
                            final_solution.push_back(placement);
                        }
                    }
                    // Add the re-optimized 'remaining' ones
                    final_solution.insert(final_solution.end(), optimized_remaining.begin(), optimized_remaining.end());

                    problem.set_current_solution(final_solution);
                    return final_solution;
                }
            }
        }

        return merged;
    }
}
