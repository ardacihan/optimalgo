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

    const int MAX_NEIGHBORS = 300;

    // Precompute occupancy and box usage
    std::unordered_map<int, std::vector<bool>> occupancy_grids;
    std::set<int> used_boxes;
    std::unordered_map<int, int> box_usage;
    int total_used_boxes = 0;

    for (const auto& rect : solution) {
        used_boxes.insert(rect.box_id);
        box_usage[rect.box_id]++;
    }
    total_used_boxes = used_boxes.size();

    // Compute occupancy grids
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

    // Find boxes with few rectangles (potential candidates to empty)
    std::vector<int> sparse_boxes;
    for (const auto& [box_id, count] : box_usage) {
        if (count <= 3 && count > 0) {
            sparse_boxes.push_back(box_id);
        }
    }

    // STRATEGY 1: For each sparse box, try to move its rectangles to other boxes
    for (int sparse_box : sparse_boxes) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        std::vector<int> rects_in_sparse_box;
        for (int i = 0; i < n; i++) {
            if (solution[i].box_id == sparse_box) {
                rects_in_sparse_box.push_back(i);
            }
        }

        for (int rect_idx : rects_in_sparse_box) {
            if (nbs.size() >= MAX_NEIGHBORS) break;

            const auto& rect = solution[rect_idx];

            for (int target_box : used_boxes) {
                if (nbs.size() >= MAX_NEIGHBORS) break;
                if (target_box == sparse_box) continue;

                auto target_grid = occupancy_grids[target_box];

                for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
                    for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                        if (target_grid[y * L + x]) continue;

                        for (int rot = 0; rot < 2; rot++) {
                            if (rot == 1 && rect.width == rect.height) continue;

                            int w = rot == 0 ? rect.width : rect.height;
                            int h = rot == 0 ? rect.height : rect.width;

                            if (x + w > L || y + h > L) {
                                continue;
                            }

                            bool can_place = true;
                            for (int dx = 0; dx < w && can_place; dx++) {
                                for (int dy = 0; dy < h && can_place; dy++) {
                                    if (target_grid[(y + dy) * L + (x + dx)]) {
                                        can_place = false;
                                    }
                                }
                            }

                            if (can_place) {
                                auto neighbor = solution;
                                neighbor[rect_idx].box_id = target_box;
                                neighbor[rect_idx].x = x;
                                neighbor[rect_idx].y = y;
                                neighbor[rect_idx].rotated = (rot == 1);
                                nbs.push_back(neighbor);
                                metadata.push_back(NeighborMetadata(rect_idx));
                                break;
                            }
                        }
                        if (nbs.size() < MAX_NEIGHBORS) break;
                    }
                    if (nbs.size() < MAX_NEIGHBORS) break;
                }
            }
        }
    }

    // STRATEGY 2: Scan for empty spaces in all boxes
    for (int target_box : used_boxes) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        auto grid = occupancy_grids[target_box];

        for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
            for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                if (!grid[y * L + x]) {
                    for (int i = 0; i < n && nbs.size() < MAX_NEIGHBORS; i++) {
                        if (solution[i].box_id == target_box) continue;

                        const auto& rect = solution[i];
                        for (int rot = 0; rot < 2; rot++) {
                            if (rot == 1 && rect.width == rect.height) continue;

                            int w = (rot == 0) ? rect.width : rect.height;
                            int h = (rot == 0) ? rect.height : rect.width;

                            if (x + w > L || y + h > L) {
                                continue;
                            }

                            bool can_place = true;
                            for (int dx = 0; dx < w && can_place; dx++) {
                                for (int dy = 0; dy < h && can_place; dy++) {
                                    int nx = x + dx;
                                    int ny = y + dy;
                                    if (nx >= L || ny >= L || grid[ny * L + nx]) {
                                        can_place = false;
                                    }
                                }
                            }

                            if (can_place) {
                                auto neighbor = solution;
                                neighbor[i].box_id = target_box;
                                neighbor[i].x = x;
                                neighbor[i].y = y;
                                neighbor[i].rotated = (rot == 1);
                                nbs.push_back(neighbor);
                                metadata.push_back(NeighborMetadata(i));
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    // STRATEGY 3: Move rectangles to most utilized box
    int max_usage = 0;
    int most_used_box = -1;
    for (const auto& [box_id, count] : box_usage) {
        if (count > max_usage) {
            max_usage = count;
            most_used_box = box_id;
        }
    }

    if (most_used_box != -1) {
        auto target_grid = occupancy_grids[most_used_box];

        for (int i = 0; i < n && nbs.size() < MAX_NEIGHBORS; i++) {
            if (solution[i].box_id == most_used_box) continue;

            const auto& rect = solution[i];

            for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
                for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                    if (target_grid[y * L + x]) continue;

                    for (int rot = 0; rot < 2; rot++) {
                        if (rot == 1 && rect.width == rect.height) continue;

                        int w = (rot == 0) ? rect.width : rect.height;
                        int h = (rot == 0) ? rect.height : rect.width;

                        if (x + w > L || y + h > L) {
                            continue;
                        }

                        bool can_place = true;
                        for (int dx = 0; dx < w && can_place; dx++) {
                            for (int dy = 0; dy < h && can_place; dy++) {
                                if (target_grid[(y + dy) * L + (x + dx)]) {
                                    can_place = false;
                                }
                            }
                        }

                        if (can_place) {
                            auto neighbor = solution;
                            neighbor[i].box_id = most_used_box;
                            neighbor[i].x = x;
                            neighbor[i].y = y;
                            neighbor[i].rotated = (rot == 1);
                            nbs.push_back(neighbor);
                            metadata.push_back(NeighborMetadata(i));
                            break;
                        }
                    }
                    if (nbs.size() < MAX_NEIGHBORS) break;
                }
                if (nbs.size() < MAX_NEIGHBORS) break;
            }
        }
    }

    // STRATEGY 4: Small position movements (1-2 units)
    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& rect = solution[rect_idx];
        int orig_x = rect.x;
        int orig_y = rect.y;
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();

        // Try moving 1 or 2 units in each direction
        for (int direction = 0; direction < 4; direction++) {
            // Try 1 unit movement
            if (nbs.size() < MAX_NEIGHBORS) {
                auto neighbor = solution;
                int new_x = orig_x;
                int new_y = orig_y;

                if (direction == 0) new_x = orig_x - 1;
                else if (direction == 1) new_x = orig_x + 1;
                else if (direction == 2) new_y = orig_y - 1;
                else if (direction == 3) new_y = orig_y + 1;

                if (new_x >= 0 && new_x + w <= L &&
                    new_y >= 0 && new_y + h <= L) {
                    if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                        neighbor[rect_idx].x = new_x;
                        neighbor[rect_idx].y = new_y;
                        nbs.push_back(neighbor);
                        metadata.push_back(NeighborMetadata(rect_idx));
                    }
                }
            }

            // Try 2 units movement
            if (nbs.size() < MAX_NEIGHBORS) {
                auto neighbor = solution;
                int new_x = orig_x;
                int new_y = orig_y;

                if (direction == 0) new_x = orig_x - 2;
                else if (direction == 1) new_x = orig_x + 2;
                else if (direction == 2) new_y = orig_y - 2;
                else if (direction == 3) new_y = orig_y + 2;

                if (new_x >= 0 && new_x + w <= L &&
                    new_y >= 0 && new_y + h <= L) {
                    if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                        neighbor[rect_idx].x = new_x;
                        neighbor[rect_idx].y = new_y;
                        nbs.push_back(neighbor);
                        metadata.push_back(NeighborMetadata(rect_idx));
                    }
                }
            }
        }

        // Diagonal moves
        int dx_vals[] = {-1, 1, -1, 1};
        int dy_vals[] = {-1, -1, 1, 1};

        for (int d = 0; d < 4 && nbs.size() < MAX_NEIGHBORS; d++) {
            int new_x = orig_x + dx_vals[d];
            int new_y = orig_y + dy_vals[d];
            
            if (new_x >= 0 && new_x + w <= L &&
                new_y >= 0 && new_y + h <= L) {
                auto neighbor = solution;
                if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(rect_idx));
                }
            }
        }
    }

    // STRATEGY 5: Move to adjacent empty spots
    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& rect = solution[rect_idx];
        int box_id = rect.box_id;
        const auto& grid = occupancy_grids[box_id];
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        int orig_x = rect.x;
        int orig_y = rect.y;

        // Try positions around the rectangle
        int positions[][2] = {
            {orig_x - w, orig_y},      // LEFT
            {orig_x + w, orig_y},      // RIGHT
            {orig_x, orig_y - h},      // ABOVE
            {orig_x, orig_y + h}       // BELOW
        };

        for (auto& pos : positions) {
            if (nbs.size() >= MAX_NEIGHBORS) break;
            
            int new_x = pos[0];
            int new_y = pos[1];
            
            if (new_x < 0 || new_y < 0 || new_x + w > L || new_y + h > L) continue;

            bool can_place = true;
            for (int dy = 0; dy < h && can_place; dy++) {
                for (int dx = 0; dx < w && can_place; dx++) {
                    int grid_x = new_x + dx;
                    int grid_y = new_y + dy;

                    bool is_original_cell = false;
                    if (grid_x >= orig_x && grid_x < orig_x + w &&
                        grid_y >= orig_y && grid_y < orig_y + h) {
                        is_original_cell = true;
                    }

                    if (!is_original_cell && grid[grid_y * L + grid_x]) {
                        can_place = false;
                    }
                }
            }

            if (can_place) {
                auto neighbor = solution;
                neighbor[rect_idx].x = new_x;
                neighbor[rect_idx].y = new_y;
                nbs.push_back(neighbor);
                metadata.push_back(NeighborMetadata(rect_idx));
            }
        }
    }

    std::cout << "Generated " << nbs.size() << " neighbors ("
              << total_used_boxes << " boxes, "
              << "with delta calculation support)\n";

    return nbs;
}