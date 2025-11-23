#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>
#include <cmath>


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

struct BoxData {
    std::vector<int> rect_indices;
    long long total_area;
    std::unique_ptr<SpatialHash> hash_grid;
    std::vector<int> occupancy_grid;
    int L;

    BoxData(int box_length, int avg_size) : total_area(0), L(box_length) {
        hash_grid = std::make_unique<SpatialHash>(L, avg_size);
        occupancy_grid.resize(L * L, -1);
    }

    BoxData(BoxData&& other) noexcept = default;
    BoxData& operator=(BoxData&& other) noexcept = default;
    BoxData(const BoxData&) = delete;
    BoxData& operator=(const BoxData&) = delete;
};

void build_box_occupancy(BoxData& data, const std::vector<RectanglePlacement>& solution) {
    std::fill(data.occupancy_grid.begin(), data.occupancy_grid.end(), -1);
    int L = data.L;

    for (int rect_idx : data.rect_indices) {
        const auto& r = solution[rect_idx];
        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int y = r.y; y < r.y + h; ++y) {
            for (int x = r.x; x < r.x + w; ++x) {
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
    if (n == 0) return nbs;

    int MAX_NEIGHBORS = 140;
    nbs.reserve(MAX_NEIGHBORS);

    int avg_size_sq = 0;
    for (const auto& r : solution) avg_size_sq += r.get_actual_width() * r.get_actual_height();
    avg_size_sq = std::max(1, avg_size_sq / n);

    // Build box data structures
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

    // Classify boxes by utilization
    std::unordered_set<int> frozen_boxes;
    std::vector<std::pair<int, double>> box_utilizations;

    for (const auto& [box_id, data] : box_data) {
        double utilization = (double)data.total_area / (double)(L * L);
        box_utilizations.push_back({box_id, utilization});
        if (utilization > 0.9) frozen_boxes.insert(box_id);
    }

    std::sort(box_utilizations.begin(), box_utilizations.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    for (auto& pair : box_data) build_box_occupancy(pair.second, solution);

    // Helper: Check collision in a DYNAMIC solution (not just the original)
    auto collides_in_solution = [&](const RectanglePlacement& r, int target_box_id, int moved_idx,
                                     const std::vector<RectanglePlacement>& current_sol) {
        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int i = 0; i < (int)current_sol.size(); i++) {
            if (i == moved_idx) continue;
            if (current_sol[i].box_id != target_box_id) continue;

            const auto& other = current_sol[i];
            int ow = other.get_actual_width();
            int oh = other.get_actual_height();

            // Check rectangle overlap
            if (!(r.x >= other.x + ow || r.x + w <= other.x ||
                  r.y >= other.y + oh || r.y + h <= other.y)) {
                return true;
            }
        }
        return false;
    };

    auto is_valid_move = [&](const RectanglePlacement& moved, int moved_idx,
                             const std::vector<RectanglePlacement>& current_sol) {
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L)
            return false;
        return !collides_in_solution(moved, moved.box_id, moved_idx, current_sol);
    };

    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    // ==================================================================
    // Strategy 1: DENSITY-DRIVEN MOVES (Primary strategy)
    // ==================================================================
    for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
        int sparse_box_id = box_utilizations[i].first;
        double sparse_util = box_utilizations[i].second;

        if (sparse_util > 0.6) break;
        if (frozen_boxes.count(sparse_box_id)) continue;

        auto sparse_it = box_data.find(sparse_box_id);
        if (sparse_it == box_data.end()) continue;
        const auto& sparse_data = sparse_it->second;

        for (int idx : sparse_data.rect_indices) {
            if (nbs.size() >= MAX_NEIGHBORS / 2) break;

            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            // Target denser boxes
            for (size_t j = box_utilizations.size() - 1; j > i; j--) {
                int target_box_id = box_utilizations[j].first;

                if (frozen_boxes.count(target_box_id)) continue;
                if (target_box_id == sparse_box_id) continue;

                auto target_it = box_data.find(target_box_id);
                if (target_it == box_data.end()) continue;
                const auto& target_data = target_it->second;
                if (target_data.total_area + rect_area > (long long)L * L * 0.95) continue;

                std::set<std::pair<int,int>> positions;
                int rect_w = rect.get_actual_width();
                int rect_h = rect.get_actual_height();

                positions.insert({0, 0});
                positions.insert({L - rect_w, 0});
                positions.insert({0, L - rect_h});

                int sample_size = std::min(15, (int)target_data.rect_indices.size());
                for (int k = 0; k < sample_size; k++) {
                    int other_idx = target_data.rect_indices[k];
                    const auto& other = solution[other_idx];
                    int ow = other.get_actual_width();
                    int oh = other.get_actual_height();

                    positions.insert({other.x + ow, other.y});
                    positions.insert({other.x - rect_w, other.y});
                    positions.insert({other.x, other.y + oh});
                    positions.insert({other.x, other.y - rect_h});
                }

                for (const auto& [px, py] : positions) {
                    if (nbs.size() >= MAX_NEIGHBORS / 2) break;

                    for (int rot = 0; rot < 2; rot++) {
                        RectanglePlacement moved = rect;
                        moved.box_id = target_box_id;
                        moved.x = px;
                        moved.y = py;
                        if (rot == 1) moved.rotated = !moved.rotated;

                        auto nb = solution;
                        nb[idx] = moved;

                        if (is_valid_move(moved, idx, nb)) {
                            if (!add_neighbor(nb)) break;
                        }
                    }
                }

                if (nbs.size() >= MAX_NEIGHBORS / 2) break;
            }
        }
    }

    // ==================================================================
    // Strategy 2: SWAP MOVES between low-utilization boxes
    // ==================================================================
    if (nbs.size() < MAX_NEIGHBORS) {
        for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
            if (box_utilizations[i].second > 0.6) break;

            for (size_t j = i + 1; j < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; j++) {
                if (box_utilizations[j].second > 0.6) break;

                int box_a = box_utilizations[i].first;
                int box_b = box_utilizations[j].first;

                auto data_a_it = box_data.find(box_a);
                auto data_b_it = box_data.find(box_b);
                if (data_a_it == box_data.end() || data_b_it == box_data.end()) continue;

                const auto& data_a = data_a_it->second;
                const auto& data_b = data_b_it->second;

                // Try swapping pairs of rectangles
                int max_tries_a = std::min(3, (int)data_a.rect_indices.size());
                int max_tries_b = std::min(3, (int)data_b.rect_indices.size());

                for (int ia = 0; ia < max_tries_a && nbs.size() < MAX_NEIGHBORS; ia++) {
                    int idx_a = data_a.rect_indices[ia];

                    for (int ib = 0; ib < max_tries_b && nbs.size() < MAX_NEIGHBORS; ib++) {
                        int idx_b = data_b.rect_indices[ib];

                        const auto& rect_a = solution[idx_a];
                        const auto& rect_b = solution[idx_b];

                        // Try swapping with rotation options
                        for (int rot_a = 0; rot_a < 2; rot_a++) {
                            for (int rot_b = 0; rot_b < 2; rot_b++) {
                                RectanglePlacement moved_a = rect_a;
                                RectanglePlacement moved_b = rect_b;

                                moved_a.box_id = box_b;
                                moved_b.box_id = box_a;

                                if (rot_a == 1) moved_a.rotated = !moved_a.rotated;
                                if (rot_b == 1) moved_b.rotated = !moved_b.rotated;

                                auto nb = solution;
                                nb[idx_a] = moved_a;
                                nb[idx_b] = moved_b;

                                if (is_valid_move(moved_a, idx_a, nb) &&
                                    is_valid_move(moved_b, idx_b, nb)) {
                                    add_neighbor(nb);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ==================================================================
    // Strategy 3: ROTATION-only moves for sparse boxes
    // ==================================================================
    if (nbs.size() < MAX_NEIGHBORS) {
        for (const auto& [box_id, util] : box_utilizations) {
            if (util > 0.5) break;
            if (nbs.size() >= MAX_NEIGHBORS) break;

            auto it = box_data.find(box_id);
            if (it == box_data.end()) continue;

            for (int idx : it->second.rect_indices) {
                if (nbs.size() >= MAX_NEIGHBORS) break;

                const auto& rect = solution[idx];
                if (rect.width == rect.height) continue; // Skip squares

                RectanglePlacement rotated = rect;
                rotated.rotated = !rotated.rotated;

                auto nb = solution;
                nb[idx] = rotated;

                if (is_valid_move(rotated, idx, nb)) {
                    add_neighbor(nb);
                }
            }
        }
    }

    //std::cout << "Generated " << nbs.size() << " smart geometry-based neighbors" << std::endl;
    return nbs;
}