//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>

// ============== Spatial Grid for Fast Collision Detection ==============

class SpatialGrid {
    int cell_size;
    int grid_width;
    std::unordered_map<int, std::vector<int>> cells;

    int get_cell_key(int cx, int cy) const {
        return cy * grid_width + cx;
    }

public:
    SpatialGrid(int box_length, int avg_rect_size) {
        cell_size = std::max(1, avg_rect_size);
        grid_width = (box_length + cell_size - 1) / cell_size + 1;
    }

    void insert(int rect_idx, const RectanglePlacement& r) {
        int x1 = r.x / cell_size;
        int y1 = r.y / cell_size;
        int x2 = (r.x + r.get_actual_width() - 1) / cell_size;
        int y2 = (r.y + r.get_actual_height() - 1) / cell_size;

        for (int cy = y1; cy <= y2; cy++) {
            for (int cx = x1; cx <= x2; cx++) {
                cells[get_cell_key(cx, cy)].push_back(rect_idx);
            }
        }
    }

    std::vector<int> query(const RectanglePlacement& r) const {
        std::set<int> result_set;
        int x1 = r.x / cell_size;
        int y1 = r.y / cell_size;
        int x2 = (r.x + r.get_actual_width() - 1) / cell_size;
        int y2 = (r.y + r.get_actual_height() - 1) / cell_size;

        for (int cy = y1; cy <= y2; cy++) {
            for (int cx = x1; cx <= x2; cx++) {
                auto it = cells.find(get_cell_key(cx, cy));
                if (it != cells.end()) {
                    result_set.insert(it->second.begin(), it->second.end());
                }
            }
        }
        return std::vector<int>(result_set.begin(), result_set.end());
    }
};

// ============== Precomputed Box Data ==============

struct BoxData {
    std::vector<int> rect_indices;
    long long total_area;
    std::unique_ptr<SpatialGrid> grid;

    BoxData(int L, int avg_size) : total_area(0) {
        grid = std::make_unique<SpatialGrid>(L, avg_size);
    }

    BoxData(BoxData&& other) noexcept = default;
    BoxData& operator=(BoxData&& other) noexcept = default;
    BoxData(const BoxData&) = delete;
    BoxData& operator=(const BoxData&) = delete;
};

// ============== IMPROVED Neighborhood Construction ==============

std::vector<std::vector<RectanglePlacement>> GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    const int MAX_NEIGHBORS = 50;


    if (n == 0) return nbs;

    // Calculate average rectangle size for spatial grid
    int avg_size = 0;
    for (const auto& r : solution) {
        avg_size += (r.get_actual_width() + r.get_actual_height()) / 2;
    }
    avg_size = std::max(1, avg_size / n);

    // Build box data structures with spatial indexing
    std::unordered_map<int, BoxData> box_data;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        auto it = box_data.find(box_id);
        if (it == box_data.end()) {
            box_data.emplace(std::piecewise_construct,
                            std::forward_as_tuple(box_id),
                            std::forward_as_tuple(L, avg_size));
            it = box_data.find(box_id);
        }
        it->second.rect_indices.push_back(i);
        it->second.total_area += solution[i].get_actual_width() * solution[i].get_actual_height();
        it->second.grid->insert(i, solution[i]);
    }

    // Fast collision check using spatial grid
    auto collides_with_any = [&](const RectanglePlacement& r, int box_id, int exclude_idx) {
        auto it = box_data.find(box_id);
        if (it == box_data.end()) return false;

        auto& grid = *it->second.grid;
        auto candidates = grid.query(r);

        for (int idx : candidates) {
            if (idx == exclude_idx) continue;
            if (solution[idx].box_id != box_id) continue;
            if (r.collides(solution[idx])) return true;
        }
        return false;
    };

    // Check if moving a rectangle creates a valid solution
    auto is_valid_move = [&](const RectanglePlacement& moved, int target_box, int moved_idx) {
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L) {
            return false;
        }
        return !collides_with_any(moved, target_box, moved_idx);
    };

    // Helper to add a neighbor
    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    nbs.reserve(MAX_NEIGHBORS);

    // ==== STRATEGY 1: Empty nearly-empty boxes ====
    std::vector<std::pair<int, int>> sparse_boxes;
    for (const auto& [box_id, data] : box_data) {
        int coverage_pct = (data.total_area * 100) / (L * L);
        if (data.rect_indices.size() <= 3 || coverage_pct < 25) {
            sparse_boxes.push_back({box_id, data.rect_indices.size()});
        }
    }
    std::sort(sparse_boxes.begin(), sparse_boxes.end(),
              [](auto& a, auto& b) { return a.second < b.second; });

    for (const auto& [sparse_box_id, _] : sparse_boxes) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        const auto& sparse_data = box_data.at(sparse_box_id);

        for (int idx : sparse_data.rect_indices) {
            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            for (const auto& [target_box_id, target_data] : box_data) {
                if (target_box_id == sparse_box_id) continue;
                if (target_data.total_area + rect_area > (long long)L * L * 0.95) continue;

                std::vector<std::pair<int,int>> try_positions;
                try_positions.push_back({0, 0});

                int max_rects = std::min(5, (int)target_data.rect_indices.size());
                for (int j = 0; j < max_rects; j++) {
                    int j_idx = target_data.rect_indices[j];
                    const auto& rect_j = solution[j_idx];

                    try_positions.push_back({rect_j.x + rect_j.get_actual_width(), rect_j.y});
                    try_positions.push_back({rect_j.x, rect_j.y + rect_j.get_actual_height()});
                    try_positions.push_back({rect_j.x - rect.get_actual_width(), rect_j.y});
                    try_positions.push_back({rect_j.x, rect_j.y - rect.get_actual_height()});
                }

                for (const auto& [nx, ny] : try_positions) {
                    if (nbs.size() >= MAX_NEIGHBORS) break;

                    RectanglePlacement moved = rect;
                    moved.box_id = target_box_id;
                    moved.x = nx;
                    moved.y = ny;

                    if (is_valid_move(moved, target_box_id, idx)) {
                        auto nb = solution;
                        nb[idx] = moved;
                        if (!add_neighbor(nb)) break;
                    }
                }
                if (nbs.size() >= MAX_NEIGHBORS) break;
            }
            if (nbs.size() >= MAX_NEIGHBORS) break;
        }
    }

    // ==== STRATEGY 2: Rotation-based neighbors ====
    std::vector<int> rotatable;
    for (int i = 0; i < n; i++) {
        const auto& r = solution[i];
        if (r.width != r.height) {
            rotatable.push_back(i);
        }
    }

    int max_rotations = std::min(20, (int)rotatable.size());
    for (int i = 0; i < max_rotations && nbs.size() < MAX_NEIGHBORS; i++) {
        int idx = rotatable[i];
        auto rotated_rect = solution[idx];
        rotated_rect.rotated = !rotated_rect.rotated;

        if (is_valid_move(rotated_rect, rotated_rect.box_id, idx)) {
            auto nb = solution;
            nb[idx] = rotated_rect;
            add_neighbor(nb);
        }

        std::vector<std::pair<int,int>> corners = {
            {0, 0},
            {L - rotated_rect.get_actual_width(), 0},
            {0, L - rotated_rect.get_actual_height()}
        };

        for (const auto& [cx, cy] : corners) {
            if (nbs.size() >= MAX_NEIGHBORS) break;
            rotated_rect.x = cx;
            rotated_rect.y = cy;
            if (is_valid_move(rotated_rect, rotated_rect.box_id, idx)) {
                auto nb = solution;
                nb[idx] = rotated_rect;
                add_neighbor(nb);
            }
        }
    }

    // ==== STRATEGY 3: Compaction moves ====
    std::vector<int> moveable_rects;
    for (int i = 0; i < n; i++) {
        if (solution[i].x > 0 || solution[i].y > 0) {
            moveable_rects.push_back(i);
        }
    }

    std::sort(moveable_rects.begin(), moveable_rects.end(), [&](int a, int b) {
        return solution[a].x + solution[a].y > solution[b].x + solution[b].y;
    });

    int max_compact = std::min(30, (int)moveable_rects.size());
    for (int i = 0; i < max_compact && nbs.size() < MAX_NEIGHBORS; i++) {
        int idx = moveable_rects[i];
        const auto& rect = solution[idx];

        std::vector<std::pair<int,int>> compact_positions;

        for (int dx = 1; dx <= rect.x && dx <= 5; dx++) {
            compact_positions.push_back({rect.x - dx, rect.y});
        }
        for (int dy = 1; dy <= rect.y && dy <= 5; dy++) {
            compact_positions.push_back({rect.x, rect.y - dy});
        }
        int min_delta = std::min(rect.x, rect.y);
        for (int d = 1; d <= min_delta && d <= 3; d++) {
            compact_positions.push_back({rect.x - d, rect.y - d});
        }

        for (const auto& [nx, ny] : compact_positions) {
            if (nbs.size() >= MAX_NEIGHBORS) break;

            RectanglePlacement moved = rect;
            moved.x = nx;
            moved.y = ny;

            if (is_valid_move(moved, rect.box_id, idx)) {
                auto nb = solution;
                nb[idx] = moved;
                add_neighbor(nb);
            }
        }
    }

    // ==== STRATEGY 4: Swap rectangles between boxes ====
    if (box_data.size() >= 2 && nbs.size() < MAX_NEIGHBORS) {
        std::vector<int> box_ids;
        for (const auto& [bid, _] : box_data) box_ids.push_back(bid);

        int max_swaps = std::min(10, (int)box_ids.size() * (int)box_ids.size());
        int swaps_tried = 0;

        for (size_t i = 0; i < box_ids.size() && swaps_tried < max_swaps; i++) {
            for (size_t j = i + 1; j < box_ids.size() && swaps_tried < max_swaps; j++) {
                int box_a = box_ids[i];
                int box_b = box_ids[j];

                const auto& data_a = box_data.at(box_a);
                const auto& data_b = box_data.at(box_b);

                if (!data_a.rect_indices.empty() && !data_b.rect_indices.empty()) {
                    int idx_a = data_a.rect_indices[0];
                    int idx_b = data_b.rect_indices[0];

                    auto rect_a = solution[idx_a];
                    auto rect_b = solution[idx_b];

                    rect_a.box_id = box_b;
                    rect_a.x = 0;
                    rect_a.y = 0;

                    rect_b.box_id = box_a;
                    rect_b.x = 0;
                    rect_b.y = 0;

                    if (is_valid_move(rect_a, box_b, idx_a) &&
                        is_valid_move(rect_b, box_a, idx_b)) {
                        auto nb = solution;
                        nb[idx_a] = rect_a;
                        nb[idx_b] = rect_b;
                        add_neighbor(nb);
                    }

                    swaps_tried++;
                }
                if (nbs.size() >= MAX_NEIGHBORS) break;
            }
            if (nbs.size() >= MAX_NEIGHBORS) break;
        }
    }

    std::cout << "Generated " << nbs.size() << " valid neighbors using multiple strategies" << std::endl;
    return nbs;
}

// ============== Solver Implementation ==============

std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();
    if (max_steps <= 0) return solution;

    auto neighbors = construct_neighbors(problem);
    if (neighbors.empty()) {
        std::cout << "No neighbors generated, stopping." << std::endl;
        return solution;
    }

    int current_obj = problem.objective(solution);

    std::vector<RectanglePlacement> best_neighbor = solution;
    int best_obj = current_obj;

    for (auto &n : neighbors) {
        int obj = problem.objective(n);
        if (obj > best_obj) {
            best_obj = obj;
            best_neighbor = n;
        }
    }

    if (best_obj > current_obj) {
        problem.set_current_solution(best_neighbor);
        std::cout << "Step " << (1201 - max_steps) << ": Improved from " << current_obj
                  << " to " << best_obj << " (Δ=" << (best_obj - current_obj)
                  << ", checked " << neighbors.size() << " neighbors)" << std::endl;
        return solve(problem, max_steps - 1);
    } else {
        std::cout << "No improvement found (current: " << current_obj
                  << ", best neighbor: " << best_obj << "), stopping early." << std::endl;
        return solution;
    }
}