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

// Helper function to check if rectangle can be placed at (x,y) in its current box
bool can_place_at_position(int rect_idx, int new_x, int new_y,
                          const std::vector<RectanglePlacement>& solution,
                          const std::unordered_map<int, std::vector<bool>>& occupancy_grids,
                          int L) {
    const auto& rect = solution[rect_idx];
    int box_id = rect.box_id;

    // Get rectangle dimensions
    int w = rect.get_actual_width();
    int h = rect.get_actual_height();

    // FIRST: Check if rectangle stays within box boundaries
    if (new_x < 0 || new_y < 0 ||
        new_x + w > L ||  // FIXED: Use w, not rect.width (accounts for rotation)
        new_y + h > L) {  // FIXED: Use h, not rect.height (accounts for rotation)
        return false; // Would exceed box boundaries
    }

    const auto& grid = occupancy_grids.at(box_id);

    // Check all cells in the new position
    for (int dy = 0; dy < h; dy++) {
        for (int dx = 0; dx < w; dx++) {
            int grid_x = new_x + dx;
            int grid_y = new_y + dy;

            // Skip checking the rectangle's original position cells
            bool is_original_cell = false;
            if (grid_x >= rect.x && grid_x < rect.x + w &&
                grid_y >= rect.y && grid_y < rect.y + h) {
                is_original_cell = true;
            }

            if (!is_original_cell && grid[grid_y * L + grid_x]) {
                return false; // Collision with another rectangle
            }
        }
    }

    return true;
}

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T)
{
    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 1000;

    // Precompute occupancy and box usage
    std::unordered_map<int, std::vector<bool>> occupancy_grids;
    std::set<int> used_boxes;

    // Track box utilization (how many rectangles in each box)
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

    // ====================================================
    // NEW: LAST STRATEGY - Move 1 or 2 places left/right/up/down if legal
    // ====================================================
    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& rect = solution[rect_idx];
        int orig_x = rect.x;
        int orig_y = rect.y;
        int w = rect.get_actual_width();  // Get actual width (accounts for rotation)
        int h = rect.get_actual_height(); // Get actual height (accounts for rotation)

        // Try moving 1 or 2 units in each direction
        for (int direction = 0; direction < 4; direction++) {
            // Try 1 unit movement
            if (nbs.size() < MAX_NEIGHBORS) {
                auto neighbor = solution;
                int new_x = orig_x;
                int new_y = orig_y;

                if (direction == 0) { // LEFT
                    new_x = orig_x - 1;
                } else if (direction == 1) { // RIGHT
                    new_x = orig_x + 1;
                } else if (direction == 2) { // UP
                    new_y = orig_y - 1;
                } else if (direction == 3) { // DOWN
                    new_y = orig_y + 1;
                }

                // Check boundaries BEFORE calling can_place_at_position
                if (new_x >= 0 && new_x + w <= L &&
                    new_y >= 0 && new_y + h <= L) {
                    if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                        neighbor[rect_idx].x = new_x;
                        neighbor[rect_idx].y = new_y;
                        nbs.push_back(neighbor);
                    }
                }
            }

            // Try 2 units movement
            if (nbs.size() < MAX_NEIGHBORS) {
                auto neighbor = solution;
                int new_x = orig_x;
                int new_y = orig_y;

                if (direction == 0) { // LEFT
                    new_x = orig_x - 2;
                } else if (direction == 1) { // RIGHT
                    new_x = orig_x + 2;
                } else if (direction == 2) { // UP
                    new_y = orig_y - 2;
                } else if (direction == 3) { // DOWN
                    new_y = orig_y + 2;
                }

                // Check boundaries
                if (new_x >= 0 && new_x + w <= L &&
                    new_y >= 0 && new_y + h <= L) {
                    if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                        neighbor[rect_idx].x = new_x;
                        neighbor[rect_idx].y = new_y;
                        nbs.push_back(neighbor);
                    }
                }
            }
        }

        // Also try diagonal moves (1 unit in both directions) - WITH BOUNDARY CHECKS
        if (nbs.size() < MAX_NEIGHBORS) {
            // UP-LEFT
            int new_x = orig_x - 1;
            int new_y = orig_y - 1;
            if (new_x >= 0 && new_x + w <= L &&
                new_y >= 0 && new_y + h <= L) {
                auto neighbor = solution;
                if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                }
            }
        }

        if (nbs.size() < MAX_NEIGHBORS) {
            // UP-RIGHT
            int new_x = orig_x + 1;
            int new_y = orig_y - 1;
            if (new_x >= 0 && new_x + w <= L &&
                new_y >= 0 && new_y + h <= L) {
                auto neighbor = solution;
                if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                }
            }
        }

        if (nbs.size() < MAX_NEIGHBORS) {
            // DOWN-LEFT
            int new_x = orig_x - 1;
            int new_y = orig_y + 1;
            if (new_x >= 0 && new_x + w <= L &&
                new_y >= 0 && new_y + h <= L) {
                auto neighbor = solution;
                if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                }
            }
        }

        if (nbs.size() < MAX_NEIGHBORS) {
            // DOWN-RIGHT
            int new_x = orig_x + 1;
            int new_y = orig_y + 1;
            if (new_x >= 0 && new_x + w <= L &&
                new_y >= 0 && new_y + h <= L) {
                auto neighbor = solution;
                if (can_place_at_position(rect_idx, new_x, new_y, solution, occupancy_grids, L)) {
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    nbs.push_back(neighbor);
                }
            }
        }
    }

    // ====================================================
    // NEW: Move to adjacent empty spots (more aggressive) - WITH FIXED BOUNDARY CHECKS
    // ====================================================
    for (int rect_idx = 0; rect_idx < n && nbs.size() < MAX_NEIGHBORS; rect_idx++) {
        const auto& rect = solution[rect_idx];
        int box_id = rect.box_id;
        const auto& grid = occupancy_grids[box_id];
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        int orig_x = rect.x;
        int orig_y = rect.y;

        // Check positions around the rectangle for empty spots

        // Position to the LEFT of current rectangle
        if (orig_x - w >= 0 && nbs.size() < MAX_NEIGHBORS) {
            bool can_place = true;
            for (int dy = 0; dy < h && can_place; dy++) {
                for (int dx = 0; dx < w && can_place; dx++) {
                    int grid_x = (orig_x - w) + dx;
                    int grid_y = orig_y + dy;

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
                neighbor[rect_idx].x = orig_x - w;
                nbs.push_back(neighbor);
            }
        }

        // Position to the RIGHT of current rectangle
        if (orig_x + w + w <= L && nbs.size() < MAX_NEIGHBORS) {
            bool can_place = true;
            for (int dy = 0; dy < h && can_place; dy++) {
                for (int dx = 0; dx < w && can_place; dx++) {
                    int grid_x = (orig_x + w) + dx;
                    int grid_y = orig_y + dy;

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
                neighbor[rect_idx].x = orig_x + w;
                nbs.push_back(neighbor);
            }
        }

        // Position ABOVE current rectangle
        if (orig_y - h >= 0 && nbs.size() < MAX_NEIGHBORS) {
            bool can_place = true;
            for (int dy = 0; dy < h && can_place; dy++) {
                for (int dx = 0; dx < w && can_place; dx++) {
                    int grid_x = orig_x + dx;
                    int grid_y = (orig_y - h) + dy;

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
                neighbor[rect_idx].y = orig_y - h;
                nbs.push_back(neighbor);
            }
        }

        // Position BELOW current rectangle
        if (orig_y + h + h <= L && nbs.size() < MAX_NEIGHBORS) {
            bool can_place = true;
            for (int dy = 0; dy < h && can_place; dy++) {
                for (int dx = 0; dx < w && can_place; dx++) {
                    int grid_x = orig_x + dx;
                    int grid_y = (orig_y + h) + dy;

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
                neighbor[rect_idx].y = orig_y + h;
                nbs.push_back(neighbor);
            }
        }
    }

    // ====================================================
    // Original strategies - NEED TO FIX BOUNDARY CHECKS HERE TOO!
    // ====================================================

    // Find boxes with few rectangles (potential candidates to empty)
    std::vector<int> sparse_boxes;
    for (const auto& [box_id, count] : box_usage) {
        if (count <= 3 && count > 0) { // Boxes with 3 or fewer rectangles
            sparse_boxes.push_back(box_id);
        }
    }

    // For each sparse box, try to move its rectangles to other boxes
    for (int sparse_box : sparse_boxes) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        // Find all rectangles in this sparse box
        std::vector<int> rects_in_sparse_box;
        for (int i = 0; i < n; i++) {
            if (solution[i].box_id == sparse_box) {
                rects_in_sparse_box.push_back(i);
            }
        }

        // Try to move each rectangle to other boxes
        for (int rect_idx : rects_in_sparse_box) {
            if (nbs.size() >= MAX_NEIGHBORS) break;

            const auto& rect = solution[rect_idx];

            for (int target_box : used_boxes) {
                if (nbs.size() >= MAX_NEIGHBORS) break;
                if (target_box == sparse_box) continue;

                auto target_grid = occupancy_grids[target_box];

                // Try to place in target box - WITH BOUNDARY CHECKS
                for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
                    for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                        if (target_grid[y * L + x]) continue;

                        for (int rot = 0; rot < 2; rot++) {
                            if (rot == 1 && rect.width == rect.height) continue;

                            int w = rot == 0 ? rect.width : rect.height;
                            int h = rot == 0 ? rect.height : rect.width;

                            // BOUNDARY CHECK: Ensure rectangle fits in box
                            if (x + w > L || y + h > L) {
                                continue; // Would exceed box boundaries
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

    for (int target_box : used_boxes) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        auto grid = occupancy_grids[target_box];

        // Scan for empty spaces
        for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
            for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                if (!grid[y * L + x]) {
                    // Found empty cell, try to place rectangles here
                    for (int i = 0; i < n && nbs.size() < MAX_NEIGHBORS; i++) {
                        if (solution[i].box_id == target_box) continue;

                        const auto& rect = solution[i];
                        for (int rot = 0; rot < 2; rot++) {
                            if (rot == 1 && rect.width == rect.height) continue;

                            int w = (rot == 0) ? rect.width : rect.height;
                            int h = (rot == 0) ? rect.height : rect.width;

                            // BOUNDARY CHECK: Ensure rectangle fits in box
                            if (x + w > L || y + h > L) {
                                continue; // Would exceed box boundaries
                            }

                            // Check if this empty spot can accommodate the rectangle
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
                                break;
                            }
                        }
                    }
                    break;
                }
            }
        }
    }

    // Find the most utilized box (most rectangles)
    int max_usage = 0;
    int most_used_box = -1;
    for (const auto& [box_id, count] : box_usage) {
        if (count > max_usage) {
            max_usage = count;
            most_used_box = box_id;
        }
    }

    // Try to move rectangles from other boxes to the most used box
    if (most_used_box != -1) {
        auto target_grid = occupancy_grids[most_used_box];

        for (int i = 0; i < n && nbs.size() < MAX_NEIGHBORS; i++) {
            if (solution[i].box_id == most_used_box) continue;

            const auto& rect = solution[i];

            // Try to find a spot in the target box
            for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
                for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                    if (target_grid[y * L + x]) continue;

                    for (int rot = 0; rot < 2; rot++) {
                        if (rot == 1 && rect.width == rect.height) continue;

                        int w = (rot == 0) ? rect.width : rect.height;
                        int h = (rot == 0) ? rect.height : rect.width;

                        // BOUNDARY CHECK: Ensure rectangle fits in box
                        if (x + w > L || y + h > L) {
                            continue; // Would exceed box boundaries
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
                            break;
                        }
                    }
                    if (nbs.size() < MAX_NEIGHBORS) break;
                }
                if (nbs.size() < MAX_NEIGHBORS) break;
            }
        }
    }

    std::cout << "Generated " << nbs.size() << " neighbors ("
              << total_used_boxes << " boxes, "
              << "with boundary-safe moves)\n";

    return nbs;
}