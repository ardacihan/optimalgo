//
// RuleBasedNeighborhoodSolver.cpp
//

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>

bool collides_with_occupancy(int x, int y, int w, int h,
                           const std::vector<int>& grid, int L) {
    for (int py = y; py < y + h; py++) {
        if (py >= L) return true;
        for (int px = x; px < x + w; px++) {
            if (px >= L) return true;
            if (grid[py * L + px] != -1) return true;
        }
    }
    return false;
}

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    std::vector<RectanglePlacement> result;

    if (rect_indices.empty()) return result;

    // Create a mock problem
    std::vector<RectanglePlacement> dummy_rects;
    RectangleFittingProblem mock_problem(L, dummy_rects);

    // Group rectangles by their original box_id (from the indices if available)
    // If we don't have original box info, we'll use the order in the list
    std::unordered_map<int, std::vector<int>> boxes_to_fill;
    std::unordered_map<int, int> rect_to_box; // Map rectangle index to box_id

    // We'll assign boxes based on the order in rect_indices
    // First rectangle in list -> box 0, next new box when we can't fit, etc.
    int current_target_box = 0;
    std::vector<int> current_box_rects;

    // Initialize occupancy grids for boxes we'll use
    std::vector<std::vector<int>> occupancy_grids;

    // Start with first rectangle
    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        // Try to place this rectangle in the current target box
        bool placed = false;

        if (current_target_box < occupancy_grids.size()) {
            // Box already has occupancy grid, try to place
            const auto& grid = occupancy_grids[current_target_box];

            // Try both orientations
            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue;
                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;
                if (w > L || h > L) continue;

                // Search for empty spot in this box
                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                            // Place it
                            result.push_back(RectanglePlacement(width, height, x, y, (rot == 1), current_target_box));
                            placed = true;

                            // Mark as occupied
                            auto& target_grid = occupancy_grids[current_target_box];
                            for (int py = y; py < y + h; py++) {
                                for (int px = x; px < x + w; px++) {
                                    target_grid[py * L + px] = 1;
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // This is the first rectangle for a new box
            // Create occupancy grid for this box
            while (current_target_box >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            // Place at (0,0) if it fits
            int w = width;
            int h = height;
            bool rotated = false;

            if (w > L || h > L) {
                // Try rotation
                if (h <= L && w <= L && width != height) {
                    std::swap(w, h);
                    rotated = true;
                } else {
                    // Minify
                    w = std::min(width, L);
                    h = std::min(height, L);
                    if (w > L) w = L;
                    if (h > L) h = L;
                }
            }

            // Place at (0,0)
            result.push_back(RectanglePlacement(width, height, 0, 0, rotated, current_target_box));
            placed = true;

            // Mark as occupied
            auto& target_grid = occupancy_grids[current_target_box];
            for (int py = 0; py < h; py++) {
                for (int px = 0; px < w; px++) {
                    target_grid[py * L + px] = 1;
                }
            }
        }

        if (!placed) {
            // Couldn't place in current box, move to next box
            current_target_box++;
            i--; // Retry this rectangle in the new box
        }
    }

    return result;
}



std::vector<std::vector<RectanglePlacement>>
RuleBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem, int T) {

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

    // THE FIX: Make sure we're checking overlaps correctly
    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {
            std::vector<int> reordered = rect_indices;
            std::swap(reordered[i], reordered[j]);

            auto new_solution = apply_greedy_placement_indexed(reordered, rect_dims, L);

            // IMPORTANT: Check if greedy placement actually placed all rectangles
            if (new_solution.size() != n) {
                std::cout << "WARNING: Greedy placement lost rectangles! Expected "
                          << n << ", got " << new_solution.size() << std::endl;
                continue; // Skip invalid solution
            }

            // Check for overlaps - but IMPORTANT: Box IDs are LOCAL to this solution
            bool has_overlap = false;
            for (int k = 0; k < n && !has_overlap; k++) {
                for (int l = k + 1; l < n && !has_overlap; l++) {
                    // Only check if rectangles are in the SAME box
                    if (new_solution[k].box_id == new_solution[l].box_id) {
                        // Use collides() which checks rectangle bounds
                        if (new_solution[k].collides(new_solution[l])) {
                            has_overlap = true;
                            std::cout << "Overlap in neighbor: rects " << k << " and " << l
                                      << " in box " << new_solution[k].box_id << std::endl;
                        }
                    }
                }
            }

            if (!has_overlap) {
                neighbors.push_back(new_solution);
            }
        }
    }

    std::cout << "Generated " << neighbors.size() << " valid neighbors" << std::endl;
    return neighbors;
}