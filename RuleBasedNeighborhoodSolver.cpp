//
// RuleBasedNeighborhoodSolver.cpp
//

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>

// ============== Greedy Placement with Index Lookup ==============

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    std::vector<RectanglePlacement> result;
    std::vector<std::vector<int>> occupancy_grids;
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

// ============== Neighborhood Construction ==============

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