//
// Optimized Solver.cpp for $10$-second performance on $1000$ rectangles.
//

#include "Solver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>
#include <cmath> // For std::ceil

// ============== 1. Spatial Hash for Candidate Pruning ==============

class SpatialHash {
    int cell_size;
    int grid_width;
    std::unordered_map<int, std::vector<int>> cells;

    int get_cell_key(int cx, int cy) const {
        return cy * grid_width + cx;
    }

public:
    SpatialHash(int box_length, int avg_rect_size) {
        cell_size = std::max(1, (int)std::round(std::sqrt(avg_rect_size * avg_rect_size)));
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

// ============== 2. Precomputed Box Data with Occupancy Grid ==============

struct BoxData {
    std::vector<int> rect_indices;
    long long total_area;
    std::unique_ptr<SpatialHash> hash_grid;

    // Occupancy Grid: L * L size vector, stores the *index* of the occupying rectangle, or -1 if free.
    std::vector<int> occupancy_grid;
    int L; // Box length stored here for easy access

    BoxData(int box_length, int avg_size) : total_area(0), L(box_length) {
        hash_grid = std::make_unique<SpatialHash>(L, avg_size);
        // Initialize occupancy grid with all free cells (-1)
        occupancy_grid.resize(L * L, -1);
    }

    // Moving semantics remain the same
    BoxData(BoxData&& other) noexcept = default;
    BoxData& operator=(BoxData&& other) noexcept = default;
    BoxData(const BoxData&) = delete;
    BoxData& operator=(const BoxData&) = delete;
};

// ============== 3. Geometry-Based Neighborhood Construction (Optimized) ==============

// Helper to quickly build the occupancy grid for a specific box.
void build_box_occupancy(BoxData& data, const std::vector<RectanglePlacement>& solution) {
    std::fill(data.occupancy_grid.begin(), data.occupancy_grid.end(), -1);
    int L = data.L;

    for (int rect_idx : data.rect_indices) {
        const auto& r = solution[rect_idx];
        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int y = r.y; y < r.y + h; ++y) {
            for (int x = r.x; x < r.x + w; ++x) {
                // Bounds check is technically not needed if placement is valid,
                // but kept for robustness.
                if (x >= 0 && x < L && y >= 0 && y < L) {
                    data.occupancy_grid[y * L + x] = rect_idx;
                }
            }
        }
    }
}


std::vector<std::vector<RectanglePlacement>> GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();

    // Adjust MAX_NEIGHBORS severely for high rectangle counts to stay within time limit
    int MAX_NEIGHBORS = n;
    //if (n > 800) MAX_NEIGHBORS = 30; // Aggressively low for 1000 rects
    //else if (n > 600) MAX_NEIGHBORS = 50;
    //else if (n > 400) MAX_NEIGHBORS = 100;
    //else if (n > 300) MAX_NEIGHBORS = 200;
    //else if (n > 200) MAX_NEIGHBORS = 400;

    if (n == 0) return nbs;

    int avg_size_sq = 0;
    for (const auto& r : solution) {
        avg_size_sq += r.get_actual_width() * r.get_actual_height();
    }
    avg_size_sq = std::max(1, avg_size_sq / n);

    std::unordered_map<int, BoxData> box_data;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        auto it = box_data.find(box_id);
        if (it == box_data.end()) {
            box_data.emplace(std::piecewise_construct,
                            std::forward_as_tuple(box_id),
                            std::forward_as_tuple(L, avg_size_sq));
            it = box_data.find(box_id);
        }
        it->second.rect_indices.push_back(i);
        it->second.total_area += solution[i].get_actual_width() * solution[i].get_actual_height();
        it->second.hash_grid->insert(i, solution[i]);
    }

    // Build the Occupancy Grid for all used boxes (CRITICAL STEP for performance)
    for (auto& pair : box_data) {
        build_box_occupancy(pair.second, solution);
    }

    // New, much faster collision check using the Occupancy Grid. O(w*h) not O(N)
    auto collides_fast = [&](const RectanglePlacement& r, int target_box_id, int moved_idx) {
        auto it = box_data.find(target_box_id);
        if (it == box_data.end()) return false;

        const auto& grid = it->second.occupancy_grid;
        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int y = r.y; y < r.y + h; y++) {
            for (int x = r.x; x < r.x + w; x++) {
                // The position in the 1D vector is y * L + x
                int occupant_idx = grid[y * L + x];

                // If the cell is occupied by ANY rectangle other than the one being moved, it collides.
                if (occupant_idx != -1 && occupant_idx != moved_idx) {
                    return true;
                }
            }
        }
        return false;
    };

    // The fast legality check
    auto is_valid_move = [&](const RectanglePlacement& moved, int target_box, int moved_idx) {
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L) {
            return false;
        }
        return !collides_fast(moved, target_box, moved_idx);
    };

    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    nbs.reserve(MAX_NEIGHBORS);

    //

    // ==== STRATEGY 1: Empty nearly-empty boxes (Contact Points) ====
    std::vector<std::pair<int, int>> sparse_boxes;
    for (const auto& [box_id, data] : box_data) {
        if (data.rect_indices.size() <= 3 || (data.total_area * 100) / (L * L) < 25) {
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

                std::set<std::pair<int,int>> contact_positions;
                int rect_w = rect.get_actual_width();
                int rect_h = rect.get_actual_height();

                // 1. Box Corners (4 candidates)
                contact_positions.insert({0, 0});
                contact_positions.insert({L - rect_w, 0});
                contact_positions.insert({0, L - rect_h});
                contact_positions.insert({L - rect_w, L - rect_h});

                // 2. Contact Points from existing rectangles
                // Keep the candidate pool small to maximize iteration speed.
                int max_candidates = std::min(15, (int)target_data.rect_indices.size());
                for (int j = 0; j < max_candidates; j++) {
                    int j_idx = target_data.rect_indices[j];
                    const auto& rect_j = solution[j_idx];
                    int rj_w = rect_j.get_actual_width();
                    int rj_h = rect_j.get_actual_height();

                    contact_positions.insert({rect_j.x + rj_w, rect_j.y});
                    contact_positions.insert({rect_j.x - rect_w, rect_j.y});
                    contact_positions.insert({rect_j.x, rect_j.y + rj_h});
                    contact_positions.insert({rect_j.x, rect_j.y - rect_h});
                }

                for (const auto& [nx, ny] : contact_positions) {
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

    // ==== STRATEGY 2: Rotation-based neighbors (No change) ====
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

        // Check placement at current (x,y)
        if (is_valid_move(rotated_rect, rotated_rect.box_id, idx)) {
            auto nb = solution;
            nb[idx] = rotated_rect;
            add_neighbor(nb);
        }

        // Try placing the rotated rectangle at the corners
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

    // ==== STRATEGY 3: Compaction moves (Simplified) ====
    std::vector<int> moveable_rects;
    for (int i = 0; i < n; i++) {
        if (solution[i].x > 0 || solution[i].y > 0) {
            moveable_rects.push_back(i);
        }
    }

    std::sort(moveable_rects.begin(), moveable_rects.end(), [&](int a, int b) {
        return (solution[a].x + solution[a].y) > (solution[b].x + solution[b].y);
    });

    int max_compact = std::min(30, (int)moveable_rects.size());
    for (int i = 0; i < max_compact && nbs.size() < MAX_NEIGHBORS; i++) {
        int idx = moveable_rects[i];
        const auto& rect = solution[idx];

        // Try X-compaction to 0 (Move left)
        RectanglePlacement moved_x = rect;
        moved_x.x = 0;
        if (is_valid_move(moved_x, rect.box_id, idx)) {
            auto nb = solution;
            nb[idx] = moved_x;
            if (!add_neighbor(nb)) break;
        }

        // Try Y-compaction to 0 (Move down)
        if (nbs.size() >= MAX_NEIGHBORS) break;
        RectanglePlacement moved_y = rect;
        moved_y.y = 0;
        if (is_valid_move(moved_y, rect.box_id, idx)) {
            auto nb = solution;
            nb[idx] = moved_y;
            if (!add_neighbor(nb)) break;
        }

        // Try XY-compaction to 0,0 corner
        if (nbs.size() >= MAX_NEIGHBORS) break;
        RectanglePlacement moved_xy = rect;
        moved_xy.x = 0;
        moved_xy.y = 0;
        if (is_valid_move(moved_xy, rect.box_id, idx)) {
            auto nb = solution;
            nb[idx] = moved_xy;
            if (!add_neighbor(nb)) break;
        }
    }

    // ==== STRATEGY 4: Swap rectangles between boxes (No change) ====
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

// ============== Solver Implementation (Unchanged, relies on fast construct_neighbors) ==============

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
        return solve(problem, max_steps - 1);
    } else {
        std::cout << "No improvement found (current: " << current_obj
                  << ", best neighbor: " << best_obj << "), stopping early." << std::endl;
        return solution;
    }
}


std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::solve_one_step(RectangleFittingProblem &problem) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

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

    problem.set_current_solution(best_neighbor);
    return best_neighbor;
}