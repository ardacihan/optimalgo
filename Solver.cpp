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
    if (n == 0) return nbs;

    int MAX_NEIGHBORS = 100;

    int avg_size_sq = 0;
    for (const auto& r : solution) avg_size_sq += r.get_actual_width() * r.get_actual_height();
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

    std::unordered_set<int> frozen_boxes;
    for (const auto& [box_id, data] : box_data) {
        double utilization = (double)data.total_area / (double)(L * L);
        if (utilization > 0.9) frozen_boxes.insert(box_id);
    }

    for (auto& pair : box_data) build_box_occupancy(pair.second, solution);

    auto collides_fast = [&](const RectanglePlacement& r, int target_box_id, int moved_idx) {
        auto it = box_data.find(target_box_id);
        if (it == box_data.end()) return false;
        const auto& grid = it->second.occupancy_grid;
        int w = r.get_actual_width();
        int h = r.get_actual_height();
        for (int y = r.y; y < r.y + h; y++)
            for (int x = r.x; x < r.x + w; x++) {
                int occupant_idx = grid[y * L + x];
                if (occupant_idx != -1 && occupant_idx != moved_idx)
                    return true;
            }
        return false;
    };

    auto is_valid_move = [&](const RectanglePlacement& moved, int target_box, int moved_idx) {
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L)
            return false;
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

    std::vector<std::pair<int, int>> sparse_boxes;
    for (const auto& [box_id, data] : box_data) {
        if (data.rect_indices.size() <= 3 || (data.total_area * 100) / (L * L) < 25)
            sparse_boxes.push_back({box_id, data.rect_indices.size()});
    }
    std::sort(sparse_boxes.begin(), sparse_boxes.end(),
              [](auto& a, auto& b) { return a.second < b.second; });

    for (const auto& [sparse_box_id, _] : sparse_boxes) {
        if (frozen_boxes.count(sparse_box_id)) continue;
        if (nbs.size() >= MAX_NEIGHBORS) break;
        const auto& sparse_data = box_data.at(sparse_box_id);

        for (int idx : sparse_data.rect_indices) {
            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            for (const auto& [target_box_id, target_data] : box_data) {
                if (frozen_boxes.count(target_box_id)) continue;
                if (target_box_id == sparse_box_id) continue;
                if (target_data.total_area + rect_area > (long long)L * L * 0.95) continue;

                std::set<std::pair<int,int>> contact_positions;
                int rect_w = rect.get_actual_width();
                int rect_h = rect.get_actual_height();

                contact_positions.insert({0, 0});
                contact_positions.insert({L - rect_w, 0});
                contact_positions.insert({0, L - rect_h});
                contact_positions.insert({L - rect_w, L - rect_h});

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

                    // Try both orientations
                    for (int rot = 0; rot < 2; ++rot) {
                        RectanglePlacement moved = rect;
                        moved.box_id = target_box_id;
                        moved.x = nx;
                        moved.y = ny;
                        if (rot == 1) moved.rotated = !moved.rotated;

                        if (is_valid_move(moved, target_box_id, idx)) {
                            auto nb = solution;
                            nb[idx] = moved;
                            if (!add_neighbor(nb)) break;
                        }
                    }
                }
                if (nbs.size() >= MAX_NEIGHBORS) break;
            }
            if (nbs.size() >= MAX_NEIGHBORS) break;
        }
    }

    std::cout << "Generated " << nbs.size() << " valid neighbors (including rotations)" << std::endl;
    return nbs;
}

// ============== Solver Implementation (Updated to use GUI config) ==============

std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
    return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
}

std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve_with_reruns(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    if (num_reruns <= 0) return solution;

    std::unordered_set<int> box_ids;
    for (const auto& placement : solution) {
        box_ids.insert(placement.box_id);
    }

    // A. Subproblem size is small enough: Apply local search
    if (box_ids.size() < max_rectangle_in_subproblem) {
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
            // Continue local search with the new, better solution
            problem.set_current_solution(best_neighbor);
            return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
        } else {
            // Local optimum reached, return the solution for merging/rerun logic in the caller
            return solution;
        }
    }
    // B. Subproblem is too large: Divide-and-Conquer
    else {
        // --- Divide ---
        auto [left, right] = splitRectanglesByBoxId(solution);
        int L = problem.get_box_length();

        RectangleFittingProblem p1(L, left);
        RectangleFittingProblem p2(L, right);

        // --- Conquer (Recursive Calls) ---
        // Pass the full num_reruns to the subproblems
        auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
        auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

        // --- Merge ---
        std::vector<RectanglePlacement> merged = solved_left;
        merged.insert(merged.end(), solved_right.begin(), solved_right.end());

        problem.set_current_solution(merged);

        // --- Rerun Logic (Filter and Re-optimize) ---
        if (num_reruns > 1) {
            auto coverage = problem.get_coverage_each_bounding_box();
            std::vector<RectanglePlacement> remaining_solution;
            int total_box_area = L * L;
            int threshold = total_box_area * 0.8;

            // Filter out rectangles in boxes that are NOT filled
            for (const auto& placement : merged) {
                if (coverage[placement.box_id] <= threshold) {
                    remaining_solution.push_back(placement);
                }
            }

            if (!remaining_solution.empty()) {
                std::unordered_set<int> remaining_box_ids;
                for (const auto& placement : remaining_solution) {
                    remaining_box_ids.insert(placement.box_id);
                }

                if (remaining_box_ids.size() >= max_rectangle_in_subproblem) {
                    RectangleFittingProblem remaining_problem(L, remaining_solution);

                    // Decrement num_reruns for the re-optimization step
                    auto optimized_remaining = solve_with_reruns(remaining_problem, num_reruns - 1, max_rectangle_in_subproblem);

                    std::vector<RectanglePlacement> final_solution;
                    // Keep the 'filled' ones
                    for (const auto& placement : merged) {
                        if (coverage[placement.box_id] > threshold) {
                            final_solution.push_back(placement);
                        }
                    }
                    // Add the re-optimized 'remaining' ones
                    final_solution.insert(final_solution.end(), optimized_remaining.begin(), optimized_remaining.end());

                    problem.set_current_solution(final_solution);
                    return final_solution;
                }
            }
        }

        return merged;
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

std::vector<RectanglePlacement> GeometryBasedNeighborhoodSolver::solve_one_step_recursive(RectangleFittingProblem &problem) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();

    auto [left,right] = splitRectanglesByBoxId(solution);

    int L = problem.get_box_length();

    RectangleFittingProblem p1 = RectangleFittingProblem(L,left);
    RectangleFittingProblem p2 = RectangleFittingProblem(L,right);

    auto left_neighborhood = construct_neighbors(p1);
    if (left_neighborhood.empty()) {
        std::cout << "No neighbors generated, stopping." << std::endl;
        return solution;
    }
    auto right_neighborhood =  construct_neighbors(p2);
    if (right_neighborhood.empty()) {
        std::cout << "No neighbors generated, stopping." << std::endl;
        return solution;
    }

    int obj_left = problem.objective(left);
    int obj_right = problem.objective(right);

    std::vector<RectanglePlacement> best_neighbor_left= left;
    std::vector<RectanglePlacement> best_neighbor_right = right;

    int best_obj_left = obj_left;
    int best_obj_right = obj_right;

    for (auto &n : left_neighborhood) {
        int obj = problem.objective(n);
        if (obj > best_obj_left) {
            best_obj_left = obj;
            best_neighbor_left = n;
        }
    }

    for (auto &n : right_neighborhood) {
        int obj = problem.objective(n);
        if (obj > best_obj_right) {
            best_obj_right = obj;
            best_neighbor_right = n;
        }
    }

    // MERGE THE SOLUTIONS
    std::vector<RectanglePlacement> merged = best_neighbor_left;
    merged.insert(merged.end(), best_neighbor_right.begin(), best_neighbor_right.end());

    problem.set_current_solution(merged);
    return merged;
}


std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::splitRectanglesByBoxId(const std::vector<RectanglePlacement>& placements) {
    std::vector<RectanglePlacement> array1, array2;

    if (placements.empty()) {
        return {array1, array2};
    }

    // Group placements by box_id
    std::unordered_map<int, std::vector<RectanglePlacement>> boxes;
    for (const auto& placement : placements) {
        boxes[placement.box_id].push_back(placement);
    }

    // Convert to vector for deterministic ordering (optional)
    std::vector<std::pair<int, std::vector<RectanglePlacement>>> boxVector(boxes.begin(), boxes.end());

    // Sort by box_id for deterministic results (optional)
    std::sort(boxVector.begin(), boxVector.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // Alternate assignment to balance the arrays
    for (size_t i = 0; i < boxVector.size(); ++i) {
        if (i % 2 == 0) {
            // Add all placements from this box to array1
            array1.insert(array1.end(), boxVector[i].second.begin(), boxVector[i].second.end());
        } else {
            // Add all placements from this box to array2
            array2.insert(array2.end(), boxVector[i].second.begin(), boxVector[i].second.end());
        }
    }

    return {array1, array2};
}