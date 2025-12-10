// ============================================================================
// FILE: solver/local_search/RuleBasedNeighborhoodSolver.cpp (OPTIMIZED 2X)
// ============================================================================
#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <random>
#include <functional>
#include <bitset>

static const int MAX_L = 10000;

bool collides_fast(const std::vector<int>& grid, int x, int y, int w, int h, int L) {
    int idx = y * L + x;
    for (int py = 0; py < h; py++) {
        const int* row_start = &grid[idx];
        for (int px = 0; px < w; px++) {
            if (row_start[px] != -1) return true;
        }
        idx += L;
    }
    return false;
}

void fill_grid_fast(std::vector<int>& grid, int x, int y, int w, int h, int L, int val) {
    int idx = y * L + x;
    for (int py = 0; py < h; py++) {
        std::fill_n(&grid[idx], w, val);
        idx += L;
    }
}

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    if (rect_indices.empty()) return {};

    std::vector<RectanglePlacement> result;
    result.reserve(rect_indices.size());

    std::vector<std::vector<int>> occupancy_grids(1, std::vector<int>(L * L, -1));
    std::vector<int> box_ids = {rect_indices[0]};
    int current_box = 0;
    int placed_count = 0;

    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        bool placed = false;

        if (current_box < occupancy_grids.size()) {
            auto& grid = occupancy_grids[current_box];

            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue;
                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;
                if (w > L || h > L) continue;

                int max_y = L - h;
                int max_x = L - w;

                for (int y = 0; y <= max_y && !placed; y++) {
                    for (int x = 0; x <= max_x && !placed; x++) {
                        if (!collides_fast(grid, x, y, w, h, L)) {
                            result.emplace_back(width, height, x, y, (rot == 1), box_ids[current_box]);
                            fill_grid_fast(grid, x, y, w, h, L, idx);
                            placed = true;
                            placed_count++;
                        }
                    }
                }
            }
        }

        if (!placed) {
            occupancy_grids.emplace_back(L * L, -1);
            box_ids.push_back(idx);
            current_box++;

            auto& grid = occupancy_grids.back();
            int w = std::min(width, L);
            int h = std::min(height, L);
            bool rotated = false;

            if (width > L || height > L) {
                if (height <= L && width <= L && width != height) {
                    std::swap(w, h);
                    rotated = true;
                }
            }

            result.emplace_back(width, height, 0, 0, rotated, idx);
            fill_grid_fast(grid, 0, 0, w, h, L, idx);
            placed_count++;
        }

        if (placed_count > 0 && placed_count % 50 == 0 && current_box < occupancy_grids.size() - 2) {
            current_box++;
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

    const auto& solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 50;
    const int MAX_SWAPS = 50;

    if (n < 2) return {};

    thread_local std::vector<std::pair<int, int>> rect_dims;
    thread_local std::vector<int> original_box_ids;
    thread_local std::vector<int> indices;

    rect_dims.resize(n);
    original_box_ids.resize(n);
    indices.resize(n);

    std::unordered_map<int, int> box_freq;
    int max_box_id = 0;

    for (int i = 0; i < n; i++) {
        rect_dims[i] = {solution[i].width, solution[i].height};
        original_box_ids[i] = solution[i].box_id;
        indices[i] = i;
        box_freq[solution[i].box_id]++;
        if (solution[i].box_id > max_box_id) max_box_id = solution[i].box_id;
    }

    std::vector<int> frequent_boxes;
    for (const auto& [box_id, freq] : box_freq) {
        if (freq > 1) frequent_boxes.push_back(box_id);
    }

    std::vector<std::pair<int, int>> swap_pairs;
    swap_pairs.reserve(MAX_SWAPS);

    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, n-1);
    std::uniform_int_distribution<int> box_dist(0, frequent_boxes.size()-1);

    for (int s = 0; s < MAX_SWAPS; s++) {
        int i = dist(rng);
        int j = dist(rng);
        if (i == j) j = (j + 1) % n;

        if (original_box_ids[i] != original_box_ids[j] || (rng() % 5) == 0) {
            swap_pairs.emplace_back(i, j);
        }
    }

    std::vector<std::vector<RectanglePlacement>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);
    metadata.resize(MAX_NEIGHBORS);

    std::unordered_set<size_t> visited_hashes;
    auto hash_solution = [](const std::vector<int>& ids) -> size_t {
        size_t h = 0;
        for (int id : ids) {
            h = h * 31 + id;
        }
        return h;
    };

    int neighbor_count = 0;
    for (const auto& [i, j] : swap_pairs) {
        if (neighbor_count >= MAX_NEIGHBORS) break;

        std::swap(indices[i], indices[j]);
        size_t h = hash_solution(indices);

        if (visited_hashes.insert(h).second) {
            auto new_solution = apply_greedy_placement_indexed(indices, rect_dims, L);
            if (new_solution.size() == n) {
                std::unordered_map<int, int> box_mapping;
                std::vector<bool> used(max_box_id + 10, false);
                int next_id = max_box_id + 1;

                for (size_t k = 0; k < new_solution.size(); k++) {
                    int new_box = new_solution[k].box_id;
                    if (box_mapping.find(new_box) == box_mapping.end()) {
                        int preferred = original_box_ids[indices[k]];
                        if (preferred <= max_box_id && !used[preferred]) {
                            box_mapping[new_box] = preferred;
                            used[preferred] = true;
                        } else {
                            while (next_id <= max_box_id && used[next_id]) next_id++;
                            if (next_id <= max_box_id) {
                                box_mapping[new_box] = next_id;
                                used[next_id] = true;
                            } else {
                                box_mapping[new_box] = max_box_id + 1;
                                max_box_id++;
                                used.resize(max_box_id + 10);
                            }
                        }
                    }
                    new_solution[k].box_id = box_mapping[new_box];
                }

                neighbors.push_back(std::move(new_solution));
                neighbor_count++;
            }
        }

        std::swap(indices[i], indices[j]);
    }

    metadata.resize(neighbors.size());
    return neighbors;
}

std::vector<RectanglePlacement> RuleBasedNeighborhoodSolver::solve_one_step(
    RectangleFittingProblem &problem, int T) {

    // Get neighbors using rule-based swaps
    std::vector<NeighborMetadata> metadata;
    auto neighbors = construct_neighbors_with_metadata(problem, T, metadata);

    // Early return if no neighbors generated
    if (neighbors.empty()) {
        return problem.get_current_solution();
    }

    // Evaluate current solution
    auto current_solution = problem.get_current_solution();
    int current_obj = problem.objective(current_solution);
    int best_obj = current_obj;
    std::vector<RectanglePlacement> best_solution = current_solution;
    bool improved = false;

    // Evaluate all neighbors and find the best one
    for (size_t i = 0; i < neighbors.size(); i++) {
        int neighbor_obj = problem.objective(neighbors[i]);

        if (neighbor_obj > best_obj) {
            best_obj = neighbor_obj;
            best_solution = neighbors[i];
            improved = true;
        }
    }

    // Update problem with best solution if improvement found
    if (improved) {
        problem.set_current_solution(best_solution);
        return best_solution;
    }

    return current_solution;
}