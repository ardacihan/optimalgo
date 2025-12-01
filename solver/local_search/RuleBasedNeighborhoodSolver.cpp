//
// RuleBasedNeighborhoodSolver.cpp
//

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    std::vector<RectanglePlacement> result;
    std::vector<std::vector<int>> occupancy_grids;
    int next_box_id = 0;

    // Initialize occupancy grid for box 0
    occupancy_grids.push_back(std::vector<int>(L * L, -1));

    auto collides = [&](int x, int y, int w, int h, int box_id) {
        // Ensure box exists
        if (box_id >= occupancy_grids.size()) {
            // This shouldn't happen - boxes are created before use
            return true;
        }

        const auto& grid = occupancy_grids[box_id];

        for (int py = y; py < y + h; py++) {
            if (py >= L || py < 0) return true;
            for (int px = x; px < x + w; px++) {
                if (px >= L || px < 0) return true;
                if (grid[py * L + px] != -1) return true;
            }
        }
        return false;
    };

    auto mark_occupied = [&](int x, int y, int w, int h, int box_id, int rect_idx) {
        // Ensure box exists (should always be true)
        while (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }

        auto& grid = occupancy_grids[box_id];
        for (int py = y; py < y + h; py++) {
            for (int px = x; px < x + w; px++) {
                // CRITICAL: Check for overlap before marking
                if (grid[py * L + px] != -1) {
                    // This should NEVER happen if collides() worked correctly
                    std::cout << "ERROR: Overlap at (" << px << "," << py
                              << ") in box " << box_id << std::endl;
                    // Mark it anyway? Or throw?
                }
                grid[py * L + px] = rect_idx;
            }
        }
    };

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
                if (rot == 1 && width == height) continue;

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
            // FIRST: Create the occupancy grid for the new box
            while (next_box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            int w = width;
            int h = height;
            bool rotated = false;

            if (w > L && h <= L) {
                rotated = true;
                std::swap(w, h);
            } else if (w <= L && h <= L) {
                // Both fit, use original
            } else {
                w = std::min(width, L);
                h = std::min(height, L);
                if (w > L) w = L;
                if (h > L) h = L;
            }

            // Try to place in the new box (which is now at index next_box_id)
            bool placed_in_new = false;
            for (int y = 0; y <= L - h && !placed_in_new; y++) {
                for (int x = 0; x <= L - w && !placed_in_new; x++) {
                    if (!collides(x, y, w, h, next_box_id)) {
                        placement = RectanglePlacement(width, height, x, y, rotated, next_box_id);
                        placed_in_new = true;
                        placed = true;
                    }
                }
            }

            if (placed) {
                next_box_id++;  // Only increment if we actually placed in this box
            } else {
                // Should never happen - we control the box creation
                std::cout << "ERROR: Couldn't place in newly created box!" << std::endl;
            }
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

std::vector<std::vector<RectanglePlacement>>
RuleBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {

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