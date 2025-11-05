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

    int MAX_NEIGHBORS = 100;

    if (n == 0) return neighbors;

    // Extract just the rectangle dimensions (order matters!)
    std::vector<RectanglePlacement> rect_order;
    for (const auto& placement : solution) {
        // Create new RectanglePlacement with only dimensions
        RectanglePlacement r(placement.width, placement.height, 0, 0, false, 0);
        rect_order.push_back(r);
    }

    // STRATEGY 1: Swap adjacent rectangles in the ordering
    for (int i = 0; i < n - 1 && neighbors.size() < MAX_NEIGHBORS; i++) {
        auto reordered = rect_order;
        std::swap(reordered[i], reordered[i + 1]);

        auto new_solution = apply_greedy_placement(reordered, L);
        neighbors.push_back(new_solution);
    }

    // STRATEGY 2: Move rectangle from position i to position j
    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {

        // Move to beginning (gets placed first = best spots)
        if (i > 0) {
            auto reordered = rect_order;
            RectanglePlacement rect = reordered[i];
            reordered.erase(reordered.begin() + i);
            reordered.insert(reordered.begin(), rect);

            auto new_solution = apply_greedy_placement(reordered, L);
            neighbors.push_back(new_solution);
        }

        // Move to end (gets placed last)
        if (i < n - 1) {
            auto reordered = rect_order;
            RectanglePlacement rect = reordered[i];
            reordered.erase(reordered.begin() + i);
            reordered.push_back(rect);

            auto new_solution = apply_greedy_placement(reordered, L);
            neighbors.push_back(new_solution);
        }

        // Move to nearby positions
        int range = std::min(5, n / 10 + 1);
        for (int j = std::max(0, i - range); j < std::min(n, i + range) && neighbors.size() < MAX_NEIGHBORS; j++) {
            if (i == j) continue;

            auto reordered = rect_order;
            RectanglePlacement rect = reordered[i];
            reordered.erase(reordered.begin() + i);
            reordered.insert(reordered.begin() + j, rect);

            auto new_solution = apply_greedy_placement(reordered, L);
            neighbors.push_back(new_solution);
        }
    }

    // STRATEGY 3: Reverse a segment
    for (int i = 0; i < n - 1 && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int len = 2; len <= std::min(5, n - i) && neighbors.size() < MAX_NEIGHBORS; len++) {
            auto reordered = rect_order;
            std::reverse(reordered.begin() + i, reordered.begin() + i + len);

            auto new_solution = apply_greedy_placement(reordered, L);
            neighbors.push_back(new_solution);
        }
    }

    // STRATEGY 4: Target rectangles in sparse boxes and move them
    std::unordered_map<int, int> box_rect_count;
    for (const auto& p : solution) {
        box_rect_count[p.box_id]++;
    }

    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        int box_id = solution[i].box_id;

        // If this rectangle is in a sparse box (≤3 rectangles), try moving it
        if (box_rect_count[box_id] <= 3) {
            // Move to front (place early to get into fuller boxes)
            if (i > 0) {
                auto reordered = rect_order;
                RectanglePlacement rect = reordered[i];
                reordered.erase(reordered.begin() + i);
                reordered.insert(reordered.begin(), rect);

                auto new_solution = apply_greedy_placement(reordered, L);
                neighbors.push_back(new_solution);
            }

            // Try placing after first quarter (good middle ground)
            int target_pos = n / 4;
            if (i != target_pos) {
                auto reordered = rect_order;
                RectanglePlacement rect = reordered[i];
                reordered.erase(reordered.begin() + i);
                reordered.insert(reordered.begin() + target_pos, rect);

                auto new_solution = apply_greedy_placement(reordered, L);
                neighbors.push_back(new_solution);
            }
        }
    }

    std::cout << "Generated " << neighbors.size() << " rule-based neighbors" << std::endl;
    return neighbors;
}

// ============== Solver Implementation ==============

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    if (max_steps <= 0) return solution;

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
        std::cout << "Improvement: " << current_obj << " -> " << best_obj << std::endl;
        problem.set_current_solution(best_neighbor);
        return solve(problem, max_steps - 1);
    } else {
        std::cout << "No improvement found, stopping." << std::endl;
        return solution;
    }
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