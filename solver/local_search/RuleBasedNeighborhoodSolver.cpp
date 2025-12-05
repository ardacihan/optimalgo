// RuleBasedNeighborhoodSolver.cpp

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>

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

    // We'll assign relative box IDs (0, 1, 2...) here
    // These will be remapped to valid/original IDs in construct_neighbors
    int current_target_box = 0;
    std::vector<std::vector<int>> occupancy_grids;

    // Start with first rectangle
    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        // Try to place this rectangle in the current target box
        bool placed = false;

        if (current_target_box < occupancy_grids.size()) {
            const auto& grid = occupancy_grids[current_target_box];

            // Try both orientations
            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue;
                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;
                if (w > L || h > L) continue;

                // Search for empty spot
                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                            result.push_back(RectanglePlacement(width, height, x, y, (rot == 1), current_target_box));
                            placed = true;
                            // Mark occupied
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
            // New Box
            while (current_target_box >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            int w = width;
            int h = height;
            bool rotated = false;

            if (w > L || h > L) {
                if (h <= L && w <= L && width != height) {
                    std::swap(w, h);
                    rotated = true;
                } else {
                    w = std::min(width, L);
                    h = std::min(height, L);
                    if (w > L) w = L;
                    if (h > L) h = L;
                }
            }

            result.push_back(RectanglePlacement(width, height, 0, 0, rotated, current_target_box));
            placed = true;

            auto& target_grid = occupancy_grids[current_target_box];
            for (int py = 0; py < h; py++) {
                for (int px = 0; px < w; px++) {
                    target_grid[py * L + px] = 1;
                }
            }
        }

        if (!placed) {
            current_target_box++;
            i--;
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

    const int MAX_NEIGHBORS = 100;

    if (n == 0) return neighbors;

    std::vector<std::pair<int, int>> rect_dims(n);
    std::vector<int> rect_indices(n);
    std::vector<int> original_box_ids(n);

    // Track the highest ID used to generate safe new IDs for splits
    int max_original_id = -1;

    for (int i = 0; i < n; i++) {
        rect_dims[i] = {solution[i].width, solution[i].height};
        rect_indices[i] = i;
        original_box_ids[i] = solution[i].box_id;
        if (solution[i].box_id > max_original_id) {
            max_original_id = solution[i].box_id;
        }
    }

    int next_free_id_base = max_original_id + 1;

    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {

            // 1. Create neighbor by swapping order
            std::vector<int> reordered = rect_indices;
            std::swap(reordered[i], reordered[j]);

            // 2. Apply greedy placement (returns box IDs 0, 1, 2...)
            auto new_solution = apply_greedy_placement_indexed(reordered, rect_dims, L);

            if (new_solution.size() != n) continue;

            // 3. REMAPPING LOGIC: Fix the Box IDs locally
            // We map (RelativeID -> RealID)
            std::map<int, int> relative_to_real_map;
            std::set<int> used_real_ids;
            int local_next_id = next_free_id_base;

            // Loop through the new solution.
            // Note: new_solution[k] corresponds to rectangle reordered[k]
            for (size_t k = 0; k < new_solution.size(); k++) {
                int relative_box_id = new_solution[k].box_id;
                int original_rect_idx = reordered[k];
                int original_id = original_box_ids[original_rect_idx];

                // If we haven't assigned a Real ID to this Relative Box yet
                if (relative_to_real_map.find(relative_box_id) == relative_to_real_map.end()) {

                    // Try to give it the original ID of this rectangle
                    if (used_real_ids.find(original_id) == used_real_ids.end()) {
                        // Original ID is available! Use it.
                        relative_to_real_map[relative_box_id] = original_id;
                        used_real_ids.insert(original_id);
                    } else {
                        // Original ID is already taken (box split?), assign a deterministic new ID
                        relative_to_real_map[relative_box_id] = local_next_id++;
                        // No need to insert into used_real_ids as local_next_id increments uniquely
                    }
                }

                // Apply the mapping
                new_solution[k].box_id = relative_to_real_map[relative_box_id];
            }

            // 4. Overlap Check (using valid IDs now)
            bool has_overlap = false;
            for (int k = 0; k < n && !has_overlap; k++) {
                for (int l = k + 1; l < n && !has_overlap; l++) {
                    if (new_solution[k].box_id == new_solution[l].box_id) {
                        if (new_solution[k].collides(new_solution[l])) {
                            has_overlap = true;
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