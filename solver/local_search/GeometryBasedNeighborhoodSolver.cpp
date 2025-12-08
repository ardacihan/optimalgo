// ============================================================================
// FILE: solver/local_search/GeometryBasedNeighborhoodSolver.cpp (UPDATED WITH DELTA)
// ============================================================================
#include "GeometryBasedNeighborhoodSolver.h"
#include "problem/BoxOccupancyUtil.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>
#include <cmath>

bool can_place_at_position(int rect_idx, int new_x, int new_y,
                          const std::vector<RectanglePlacement>& solution,
                          const std::unordered_map<int, std::vector<bool>>& occupancy_grids,
                          int L) {
    const auto& rect = solution[rect_idx];
    int box_id = rect.box_id;

    int w = rect.get_actual_width();
    int h = rect.get_actual_height();

    if (new_x < 0 || new_y < 0 ||
        new_x + w > L ||
        new_y + h > L) {
        return false;
    }

    const auto& grid = occupancy_grids.at(box_id);

    for (int dy = 0; dy < h; dy++) {
        for (int dx = 0; dx < w; dx++) {
            int grid_x = new_x + dx;
            int grid_y = new_y + dy;

            bool is_original_cell = false;
            if (grid_x >= rect.x && grid_x < rect.x + w &&
                grid_y >= rect.y && grid_y < rect.y + h) {
                is_original_cell = true;
            }

            if (!is_original_cell && grid[grid_y * L + grid_x]) {
                return false;
            }
        }
    }

    return true;
}

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T)
{
    std::vector<NeighborMetadata> dummy_metadata;
    return construct_neighbors_with_metadata(problem, T, dummy_metadata);
}

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
    RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata)
{
    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 400;

    std::unordered_map<int, std::vector<bool>> occupancy_grids;
    std::set<int> used_boxes;

    for (const auto& rect : solution) {
        used_boxes.insert(rect.box_id);
    }

    auto compute_occupancy_grid = [&](int box_id) {
        std::vector<bool> grid(L * L, false);
        for (const auto& rect : solution) {
            if (rect.box_id == box_id) {
                int w = rect.get_actual_width();
                int h = rect.get_actual_height();
                for (int x = rect.x; x < rect.x + w; x++) {
                    for (int y = rect.y; y < rect.y + h; y++) {
                        if (x < L && y < L) {
                            grid[y * L + x] = true;
                        }
                    }
                }
            }
        }
        return grid;
    };

    for (int box_id : used_boxes) {
        occupancy_grids[box_id] = compute_occupancy_grid(box_id);
    }

    auto can_place = [&](int rect_idx, int new_x, int new_y, bool new_rotated, int target_box) {
        const auto& rect = solution[rect_idx];
        int w = new_rotated ? rect.height : rect.width;
        int h = new_rotated ? rect.width : rect.height;

        if (new_x < 0 || new_y < 0 || new_x + w > L || new_y + h > L) {
            return false;
        }

        const auto& grid = occupancy_grids.at(target_box);

        for (int dy = 0; dy < h; dy++) {
            for (int dx = 0; dx < w; dx++) {
                int grid_x = new_x + dx;
                int grid_y = new_y + dy;

                if (grid[grid_y * L + grid_x]) {
                    if (rect.box_id == target_box) {
                        int orig_w = rect.get_actual_width();
                        int orig_h = rect.get_actual_height();
                        bool is_original_cell = (grid_x >= rect.x && grid_x < rect.x + orig_w &&
                                                grid_y >= rect.y && grid_y < rect.y + orig_h);
                        if (!is_original_cell) {
                            return false;
                        }
                    } else {
                        return false;
                    }
                }
            }
        }
        return true;
    };

    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& moving_rect = solution[rect_idx];

        for (int other_idx = 0; other_idx < n && nbs.size() < MAX_NEIGHBORS; other_idx++) {
            if (rect_idx == other_idx) continue;

            const auto& other_rect = solution[other_idx];
            int target_box = other_rect.box_id;

            int w = moving_rect.get_actual_width();
            int h = moving_rect.get_actual_height();

            std::vector<std::pair<int, int>> positions = {
                {other_rect.x - w, other_rect.y},
                {other_rect.x + other_rect.get_actual_width(), other_rect.y},
                {other_rect.x, other_rect.y - h},
                {other_rect.x, other_rect.y + other_rect.get_actual_height()}
            };

            for (const auto& [new_x, new_y] : positions) {
                if (nbs.size() >= MAX_NEIGHBORS) break;

                if (can_place(rect_idx, new_x, new_y, moving_rect.rotated, target_box)) {
                    auto neighbor = solution;
                    neighbor[rect_idx].box_id = target_box;
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
        }
    }

    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& rect = solution[rect_idx];
        int box_id = rect.box_id;
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();

        for (int direction = 0; direction < 4; direction++) {
            if (nbs.size() >= MAX_NEIGHBORS) break;

            int max_shift = 0;

            if (direction == 0) {
                for (int shift = 1; shift <= rect.x; shift++) {
                    if (!can_place(rect_idx, rect.x - shift, rect.y, rect.rotated, box_id)) {
                        max_shift = shift - 1;
                        break;
                    }
                    max_shift = shift;
                }
                if (max_shift > 0) {
                    auto neighbor = solution;
                    neighbor[rect_idx].x = rect.x - max_shift;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
            else if (direction == 1) {
                for (int shift = 1; shift <= L - (rect.x + w); shift++) {
                    if (!can_place(rect_idx, rect.x + shift, rect.y, rect.rotated, box_id)) {
                        max_shift = shift - 1;
                        break;
                    }
                    max_shift = shift;
                }
                if (max_shift > 0) {
                    auto neighbor = solution;
                    neighbor[rect_idx].x = rect.x + max_shift;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
            else if (direction == 2) {
                for (int shift = 1; shift <= rect.y; shift++) {
                    if (!can_place(rect_idx, rect.x, rect.y - shift, rect.rotated, box_id)) {
                        max_shift = shift - 1;
                        break;
                    }
                    max_shift = shift;
                }
                if (max_shift > 0) {
                    auto neighbor = solution;
                    neighbor[rect_idx].y = rect.y - max_shift;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
            else if (direction == 3) {
                for (int shift = 1; shift <= L - (rect.y + h); shift++) {
                    if (!can_place(rect_idx, rect.x, rect.y + shift, rect.rotated, box_id)) {
                        max_shift = shift - 1;
                        break;
                    }
                    max_shift = shift;
                }
                if (max_shift > 0) {
                    auto neighbor = solution;
                    neighbor[rect_idx].y = rect.y + max_shift;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
        }
    }

    return nbs;
}