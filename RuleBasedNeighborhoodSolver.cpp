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
#include <unordered_set>

// ============== Simplified Greedy Placement Rule with Index Lookup ==============

std::vector<RectanglePlacement> RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    std::vector<RectanglePlacement> result;
    std::vector<std::vector<int>> occupancy_grids; // Use vector instead of unordered_map for predictable box IDs
    int next_box_id = 0;

    auto collides = [&](int x, int y, int w, int h, int box_id) {
        if (box_id >= occupancy_grids.size()) return true;

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
        while (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }

        auto& grid = occupancy_grids[box_id];
        for (int py = y; py < y + h; py++) {
            for (int px = x; px < x + w; px++) {
                grid[py * L + px] = rect_idx;
            }
        }
    };

    // GREEDY PLACEMENT: For each rectangle index in order, find first valid spot
    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        RectanglePlacement placement(width, height, 0, 0, false, -1);
        bool placed = false;

        // Try to place in existing boxes first
        for (int box_id = 0; box_id < next_box_id && !placed; box_id++) {
            // Try both orientations
            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue; // Skip rotation for squares

                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;

                if (w > L || h > L) continue;

                // Try bottom-left positions
                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides(x, y, w, h, box_id)) {
                            placement = RectanglePlacement(width, height, x, y, (rot == 1), box_id);
                            placed = true;
                        }
                    }
                }
            }
        }

        // If no existing box works, create new box
        if (!placed) {
            // Simplified placement logic
            int w = width;
            int h = height;
            bool rotated = false;

            // Try rotation if needed
            if (w > L && h <= L) {
                rotated = true;
                std::swap(w, h);
            } else if (h > L && w <= L) {
                // Keep original orientation
            } else if (w <= L && h <= L) {
                // Both fit, use original
            } else {
                // Neither fits, use minimum dimensions that fit
                w = std::min(width, L);
                h = std::min(height, L);
                if (w > L) w = L;
                if (h > L) h = L;
            }

            placement = RectanglePlacement(width, height, 0, 0, rotated, next_box_id++);
            placed = true;
        }

        if (placed) {
            result.push_back(placement);
            mark_occupied(placement.x, placement.y,
                         placement.get_actual_width(), placement.get_actual_height(),
                         placement.box_id, i);
        }
    }

    return result;
}

// ============== Neighborhood: Swap indices instead of copying rectangles ==============

std::vector<std::vector<RectanglePlacement>> RuleBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> neighbors;
    auto solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 50;

    if (n == 0) return neighbors;

    // Build lookup table: index -> (width, height)
    std::vector<std::pair<int, int>> rect_dims(n);
    std::vector<int> rect_indices(n);

    for (int i = 0; i < n; i++) {
        rect_dims[i] = {solution[i].width, solution[i].height};
        rect_indices[i] = i;
    }

    // Swap indices (cheap!) instead of copying rectangles
    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {
            std::vector<int> reordered = rect_indices;
            std::swap(reordered[i], reordered[j]);

            auto new_solution = apply_greedy_placement_indexed(reordered, rect_dims, L);
            neighbors.push_back(new_solution);
        }
    }

    std::cout << "Generated " << neighbors.size() << " neighbors" << std::endl;
    return neighbors;
}


// Helper function to remap box IDs to avoid conflicts
std::vector<RectanglePlacement> remap_box_ids(const std::vector<RectanglePlacement>& solution, int start_id) {
    std::vector<RectanglePlacement> result;
    std::unordered_map<int, int> id_mapping;
    int next_id = start_id;

    for (const auto& placement : solution) {
        if (id_mapping.find(placement.box_id) == id_mapping.end()) {
            id_mapping[placement.box_id] = next_id++;
        }
        int new_box_id = id_mapping[placement.box_id];
        result.emplace_back(placement.width, placement.height, placement.x, placement.y,
                           placement.rotated, new_box_id);
    }

    return result;
}

// ============== Solver Implementation ==============

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps, int max_rectangle_in_subproblem) {
    return solve_with_reruns(problem, max_steps, max_rectangle_in_subproblem);
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

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::solve_with_reruns(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
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
            problem.set_current_solution(best_neighbor);
            return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
        } else {
            return solution;
        }
    }
    // B. Subproblem is too large: Divide-and-Conquer
    else {
        auto [left, right] = splitRectanglesByBoxId(solution);
        int L = problem.get_box_length();

        RectangleFittingProblem p1(L, left);
        RectangleFittingProblem p2(L, right);

        auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
        auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

        // REMAP BOX IDs to avoid conflicts when merging
        int max_left_id = 0;
        for (const auto& placement : solved_left) {
            max_left_id = std::max(max_left_id, placement.box_id);
        }
        auto remapped_right = remap_box_ids(solved_right, max_left_id + 1);

        std::vector<RectanglePlacement> merged = solved_left;
        merged.insert(merged.end(), remapped_right.begin(), remapped_right.end());

        problem.set_current_solution(merged);

        if (num_reruns > 1) {
            auto coverage = problem.get_coverage_each_bounding_box();
            std::vector<RectanglePlacement> remaining_solution;
            int total_box_area = L * L;
            int threshold = total_box_area * 0.7;

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
                    auto optimized_remaining = solve_with_reruns(remaining_problem, num_reruns - 1, max_rectangle_in_subproblem);

                    std::vector<RectanglePlacement> final_solution;
                    for (const auto& placement : merged) {
                        if (coverage[placement.box_id] > threshold) {
                            final_solution.push_back(placement);
                        }
                    }

                    // Remap box IDs for the optimized remaining part
                    int final_max_id = 0;
                    for (const auto& placement : final_solution) {
                        final_max_id = std::max(final_max_id, placement.box_id);
                    }
                    auto remapped_optimized = remap_box_ids(optimized_remaining, final_max_id + 1);

                    final_solution.insert(final_solution.end(), remapped_optimized.begin(), remapped_optimized.end());

                    problem.set_current_solution(final_solution);
                    return final_solution;
                }
            }
        }

        return merged;
    }
}