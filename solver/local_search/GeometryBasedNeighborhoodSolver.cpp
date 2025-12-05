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

                // Try to place in target box
                for (int x = 0; x < L && nbs.size() < MAX_NEIGHBORS; x++) {
                    for (int y = 0; y < L && nbs.size() < MAX_NEIGHBORS; y++) {
                        if (target_grid[y * L + x]) continue;

                        for (int rot = 0; rot < 2; rot++) {
                            if (rot == 1 && rect.width == rect.height) continue;

                            int w = rect.get_actual_width();
                            int h = rect.get_actual_height();

                            if (x + w > L || y + h > L) continue;

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

                        if (x + w > L || y + h > L) continue;

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



    std::cout << "Generated " << nbs.size() << " minimal-box neighbors ("
              << total_used_boxes << " boxes currently used)\n";

    return nbs;
}

