#include "GeometryBasedNeighborhoodSolver.h"
#include "problem/BoxOccupancyUtil.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <set>

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
    std::vector<std::vector<RectanglePlacement>> neighbors;
    metadata.clear();

    const auto& solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();

    if (n == 0) return neighbors;

    const int MAX_NEIGHBORS = 1800;
    neighbors.reserve(MAX_NEIGHBORS);

    std::unordered_map<int, std::vector<bool>> occupancy_grids;
    std::unordered_map<int, int> box_occupancy_count;
    std::set<int> used_boxes;

    for (const auto& rect : solution) used_boxes.insert(rect.box_id);

    for (int box_id : used_boxes) {
        std::vector<bool> grid(L * L, false);
        int count = 0;
        for (const auto& rect : solution) {
            if (rect.box_id != box_id) continue;
            int w = rect.get_actual_width();
            int h = rect.get_actual_height();
            for (int y = rect.y; y < rect.y + h && y < L; y++) {
                for (int x = rect.x; x < rect.x + w && x < L; x++) {
                    grid[y * L + x] = true;
                    count++;
                }
            }
        }
        occupancy_grids[box_id] = std::move(grid);
        box_occupancy_count[box_id] = count;
    }

    auto can_place = [&](int rect_idx, int new_x, int new_y, bool new_rotated, int target_box) -> bool {
        const auto& rect = solution[rect_idx];
        int w = new_rotated ? rect.height : rect.width;
        int h = new_rotated ? rect.width : rect.height;
        if (new_x < 0 || new_y < 0 || new_x + w > L || new_y + h > L) return false;
        const auto& grid = occupancy_grids.at(target_box);
        for (int dy = 0; dy < h; dy++) {
            for (int dx = 0; dx < w; dx++) {
                int gx = new_x + dx;
                int gy = new_y + dy;
                if (grid[gy * L + gx]) {
                    if (rect.box_id == target_box) {
                        int ow = rect.get_actual_width();
                        int oh = rect.get_actual_height();
                        bool is_original = gx >= rect.x && gx < rect.x + ow &&
                                           gy >= rect.y && gy < rect.y + oh;
                        if (!is_original) return false;
                    } else return false;
                }
            }
        }
        return true;
    };

    std::vector<int> rect_indices(n);
    std::iota(rect_indices.begin(), rect_indices.end(), 0);

    std::sort(rect_indices.begin(), rect_indices.end(), [&](int a, int b) {
        int occ_a = box_occupancy_count[solution[a].box_id];
        int occ_b = box_occupancy_count[solution[b].box_id];
        if (occ_a != occ_b) return occ_a < occ_b;
        int area_a = solution[a].width * solution[a].height;
        int area_b = solution[b].width * solution[b].height;
        return area_a > area_b;
    });

    std::vector<int> target_boxes(used_boxes.begin(), used_boxes.end());
    std::sort(target_boxes.begin(), target_boxes.end(), [&](int a, int b) {
        if (box_occupancy_count[a] != box_occupancy_count[b])
            return box_occupancy_count[a] > box_occupancy_count[b];
        return a < b;
    });

    if (target_boxes.size() > 3) {
        int offset = rand() % 3;
        std::rotate(target_boxes.begin(),
                    target_boxes.begin() + offset,
                    target_boxes.end());
    }

    for (int rect_idx : rect_indices) {
        if (neighbors.size() >= MAX_NEIGHBORS) break;

        const auto& moving_rect = solution[rect_idx];

        for (int target_box : target_boxes) {
            if (neighbors.size() >= MAX_NEIGHBORS) break;
            if (target_box == moving_rect.box_id) continue;

            for (int other_idx = 0; other_idx < n; other_idx++) {
                if (neighbors.size() >= MAX_NEIGHBORS) break;
                if (solution[other_idx].box_id != target_box) continue;

                const auto& other = solution[other_idx];
                int w = moving_rect.get_actual_width();
                int h = moving_rect.get_actual_height();

                std::vector<std::pair<int, int>> positions = {
                    {other.x - w, other.y},
                    {other.x + other.get_actual_width(), other.y},
                    {other.x, other.y - h},
                    {other.x, other.y + other.get_actual_height()}
                };

                for (const auto& [new_x, new_y] : positions) {
                    if (can_place(rect_idx, new_x, new_y, moving_rect.rotated, target_box)) {
                        auto neighbor = solution;
                        neighbor[rect_idx].box_id = target_box;
                        neighbor[rect_idx].x = new_x;
                        neighbor[rect_idx].y = new_y;
                        neighbors.push_back(neighbor);
                        metadata.push_back(NeighborMetadata(rect_idx));
                        break;
                    }
                }

                if (moving_rect.width != moving_rect.height) {
                    int rot_w = moving_rect.height;
                    int rot_h = moving_rect.width;

                    std::vector<std::pair<int, int>> rot_positions = {
                        {other.x - rot_w, other.y},
                        {other.x + other.get_actual_width(), other.y},
                        {other.x, other.y - rot_h},
                        {other.x, other.y + other.get_actual_height()}
                    };

                    for (const auto& [new_x, new_y] : rot_positions) {
                        if (can_place(rect_idx, new_x, new_y, !moving_rect.rotated, target_box)) {
                            auto neighbor = solution;
                            neighbor[rect_idx].box_id = target_box;
                            neighbor[rect_idx].x = new_x;
                            neighbor[rect_idx].y = new_y;
                            neighbor[rect_idx].rotated = !moving_rect.rotated;
                            neighbors.push_back(neighbor);
                            metadata.push_back(NeighborMetadata(rect_idx));
                            break;
                        }
                    }
                }
            }
        }
    }

    for (int rect_idx : rect_indices) {
        if (neighbors.size() >= MAX_NEIGHBORS) break;

        const auto& rect = solution[rect_idx];
        int box_id = rect.box_id;
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();

        int max_left = 0;
        for (int shift = 1; shift <= rect.x; shift++) {
            if (!can_place(rect_idx, rect.x - shift, rect.y, rect.rotated, box_id)) break;
            max_left = shift;
        }
        if (max_left > 0) {
            auto neighbor = solution;
            neighbor[rect_idx].x = rect.x - max_left;
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(rect_idx));
        }

        int max_right = 0;
        for (int shift = 1; shift <= L - (rect.x + w); shift++) {
            if (!can_place(rect_idx, rect.x + shift, rect.y, rect.rotated, box_id)) break;
            max_right = shift;
        }
        if (max_right > 0) {
            auto neighbor = solution;
            neighbor[rect_idx].x = rect.x + max_right;
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(rect_idx));
        }

        int max_up = 0;
        for (int shift = 1; shift <= rect.y; shift++) {
            if (!can_place(rect_idx, rect.x, rect.y - shift, rect.rotated, box_id)) break;
            max_up = shift;
        }
        if (max_up > 0) {
            auto neighbor = solution;
            neighbor[rect_idx].y = rect.y - max_up;
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(rect_idx));
        }

        int max_down = 0;
        for (int shift = 1; shift <= L - (rect.y + h); shift++) {
            if (!can_place(rect_idx, rect.x, rect.y + shift, rect.rotated, box_id)) break;
            max_down = shift;
        }
        if (max_down > 0) {
            auto neighbor = solution;
            neighbor[rect_idx].y = rect.y + max_down;
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(rect_idx));
        }
    }

    return neighbors;
}


std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::solve_one_step(RectangleFittingProblem &problem, int T) {
    std::vector<NeighborMetadata> dummy_metadata;
    auto neighbors = construct_neighbors_with_metadata(problem, T, dummy_metadata);
    int best_obj = problem.objective(problem.get_current_solution());
    std::vector<RectanglePlacement> best_solution = problem.get_current_solution();
    bool improved = false;

    for (int i = 0; i < neighbors.size(); i++) {
        // Temporarily apply the neighbor solution
        std::vector<RectanglePlacement> original_solution = problem.get_current_solution();
        problem.set_current_solution(neighbors[i]);

        // Calculate objective for this neighbor
        int neighbor_obj = problem.objective(neighbors[i]);

        // Check if this neighbor is better
        if (neighbor_obj > best_obj) {
            best_obj = neighbor_obj;
            best_solution = neighbors[i];
            improved = true;
        }

        // Restore original solution to continue exploring neighbors
        problem.set_current_solution(original_solution);
    }

    // If we found an improvement, update the problem with the best solution
    if (improved) {
        problem.set_current_solution(best_solution);
    }

    return problem.get_current_solution();
}