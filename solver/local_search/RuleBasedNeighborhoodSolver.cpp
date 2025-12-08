// ============================================================================
// FILE: solver/local_search/RuleBasedNeighborhoodSolver.cpp (UPDATED WITH DELTA)
// ============================================================================
#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <chrono>
#include <cstdint>

class UniqueIDGenerator {
private:
    static uint64_t counter;

public:
    static uint64_t getUniqueID() {
        auto now = std::chrono::high_resolution_clock::now();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        return (nanos << 16) | (counter++ & 0xFFFF);
    }
};

uint64_t UniqueIDGenerator::counter = 0;

bool collides_with_occupancy_rb(int x, int y, int w, int h,
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

    int current_target_box = 0;
    std::vector<std::vector<int>> occupancy_grids;

    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        bool placed = false;

        if (current_target_box < occupancy_grids.size()) {
            const auto& grid = occupancy_grids[current_target_box];

            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue;
                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;
                if (w > L || h > L) continue;

                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides_with_occupancy_rb(x, y, w, h, grid, L)) {
                            result.push_back(RectanglePlacement(width, height, x, y, (rot == 1), current_target_box));
                            placed = true;

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
    std::vector<NeighborMetadata> dummy_metadata;
    return construct_neighbors_with_metadata(problem, T, dummy_metadata);
}

std::vector<std::vector<RectanglePlacement>>
RuleBasedNeighborhoodSolver::construct_neighbors_with_metadata(
    RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) {

    std::vector<std::vector<RectanglePlacement>> neighbors;
    auto solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 200;

    if (n == 0) return neighbors;

    std::vector<std::pair<int, int>> rect_dims(n);
    std::vector<int> rect_indices(n);
    std::vector<int> original_box_ids(n);

    std::set<int> used_box_ids;
    for (int i = 0; i < n; i++) {
        rect_dims[i] = {solution[i].width, solution[i].height};
        rect_indices[i] = i;
        original_box_ids[i] = solution[i].box_id;
        used_box_ids.insert(solution[i].box_id);
    }

    int max_box_id = 0;
    for (int box_id : used_box_ids) {
        if (box_id > max_box_id) max_box_id = box_id;
    }
    int next_box_id = max_box_id + 1;

    // Generate neighbors by swapping rectangle order
    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {

            // Create neighbor by swapping order
            std::vector<int> reordered = rect_indices;
            std::swap(reordered[i], reordered[j]);

            // Apply greedy placement
            auto new_solution = apply_greedy_placement_indexed(reordered, rect_dims, L);

            if (new_solution.size() != n) continue;

            // Remap box IDs
            std::map<int, int> relative_to_real_map;
            std::set<int> assigned_real_ids;

            for (size_t k = 0; k < new_solution.size(); k++) {
                int relative_box_id = new_solution[k].box_id;
                int original_rect_idx = reordered[k];
                int original_id = original_box_ids[original_rect_idx];

                if (relative_to_real_map.find(relative_box_id) == relative_to_real_map.end()) {
                    if (assigned_real_ids.find(original_id) == assigned_real_ids.end()) {
                        relative_to_real_map[relative_box_id] = original_id;
                        assigned_real_ids.insert(original_id);
                    } else {
                        relative_to_real_map[relative_box_id] = next_box_id++;
                        assigned_real_ids.insert(relative_to_real_map[relative_box_id]);
                    }
                }

                new_solution[k].box_id = relative_to_real_map[relative_box_id];
            }

            neighbors.push_back(new_solution);

            // IMPORTANT: Rule-based solver generates entirely new solutions through
            // permutation + greedy placement, so we CANNOT use delta calculation.
            // Every neighbor is a complete re-arrangement.
            metadata.push_back(NeighborMetadata()); // No delta support
        }
    }
    return neighbors;
}