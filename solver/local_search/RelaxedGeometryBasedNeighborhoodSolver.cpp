#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include "problem/BoxOccupancyUtil.h"
#include <cmath>
#include <random>
#include <set>
#include <algorithm>

static int current_relaxed_temperature = 1000;

void reset_relaxed_temperature() {
    current_relaxed_temperature = 1000;
}

std::vector<std::vector<RectanglePlacement>> RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    auto neighbors = construct_overlapping_neighbors(problem, current_relaxed_temperature);

    // Decrease temperature after generating neighbors
    if (current_relaxed_temperature > 0) {
        current_relaxed_temperature = std::max(0, current_relaxed_temperature - 50);
    }

    return neighbors;
}

std::vector<std::vector<RectanglePlacement>> RelaxedGeometryBasedNeighborhoodSolver::construct_overlapping_neighbors(
    RectangleFittingProblem &problem, int T) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    // 1. CALCULATE ALLOWED OVERLAP BASED ON TEMPERATURE (T)
    // As T decreases, allowed_overlap approaches 0 (Hard constraints)
    long long allowed_overlap_area = calculate_max_overlap_area(T, L);

    int MAX_NEIGHBORS = 100;
    nbs.reserve(MAX_NEIGHBORS);

    std::random_device rd;
    std::mt19937 gen(rd());

    // --- Box Data Setup (Same as before) ---
    std::unordered_map<int, BoxData> box_data;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        auto it = box_data.find(box_id);
        if (it == box_data.end()) {
            box_data.emplace(std::piecewise_construct,
                            std::forward_as_tuple(box_id),
                            std::forward_as_tuple(L, 100));
            it = box_data.find(box_id);
        }
        it->second.rect_indices.push_back(i);
        it->second.total_area += (long long)solution[i].get_actual_width() * solution[i].get_actual_height();
    }

    std::vector<std::pair<int, double>> box_utilizations;
    for (const auto& [box_id, data] : box_data) {
        double utilization = (double)data.total_area / (double)(L * L);
        box_utilizations.push_back({box_id, utilization});
    }

    std::sort(box_utilizations.begin(), box_utilizations.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    // 2. NEW HELPER: Calculate Intersection Area
    auto calculate_intersection = [&](const RectanglePlacement& r1, const RectanglePlacement& r2) -> long long {
        int x_overlap = std::max(0, std::min(r1.x + r1.get_actual_width(), r2.x + r2.get_actual_width()) - std::max(r1.x, r2.x));
        int y_overlap = std::max(0, std::min(r1.y + r1.get_actual_height(), r2.y + r2.get_actual_height()) - std::max(r1.y, r2.y));
        return (long long)x_overlap * y_overlap;
    };

    // 3. UPDATED CHECKER: Returns true if overlap is within allowed limit
    auto is_valid_relaxed = [&](const RectanglePlacement& r, int box_id,
                                const std::vector<RectanglePlacement>& sol, int skip_idx) -> bool {
        long long current_overlap = 0;

        for (int i = 0; i < (int)sol.size(); i++) {
            if (i == skip_idx) continue;
            if (sol[i].box_id != box_id) continue;

            // Check if they intersect
            current_overlap += calculate_intersection(r, sol[i]);

            // Optimization: Fail early if we exceed the limit
            if (current_overlap > allowed_overlap_area) return false;
        }
        return true;
    };

    // --- Generation Loops ---

    // LOOP 1: Move from sparse to target
    for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
        // ... (existing selection logic) ...
        int sparse_box_id = box_utilizations[i].first;
        if (box_utilizations[i].second > 0.7) break;

        auto sparse_it = box_data.find(sparse_box_id);
        if (sparse_it == box_data.end()) continue;
        auto indices = sparse_it->second.rect_indices;
        std::shuffle(indices.begin(), indices.end(), gen);

        int max_rects_to_move = std::min(5, (int)indices.size());

        for (int rect_num = 0; rect_num < max_rects_to_move && nbs.size() < MAX_NEIGHBORS; rect_num++) {
            int idx = indices[rect_num];
            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            for (size_t j = box_utilizations.size() - 1; j > i && nbs.size() < MAX_NEIGHBORS; j--) {
                int target_box_id = box_utilizations[j].first;
                // ... (existing target checks) ...
                if (target_box_id == sparse_box_id) continue;
                auto target_it = box_data.find(target_box_id);
                if (target_it == box_data.end()) continue;

                // Allow slightly more aggressive filling if T is high
                if (target_it->second.total_area + rect_area > (long long)L * L * 0.95) continue;

                for (int attempt = 0; attempt < 15 && nbs.size() < MAX_NEIGHBORS; attempt++) {
                    for (int rot = 0; rot < 2; rot++) {
                        RectanglePlacement moved = rect;
                        moved.box_id = target_box_id;
                        if (rot == 1) moved.rotated = !moved.rotated;

                        int max_x = L - moved.get_actual_width();
                        int max_y = L - moved.get_actual_height();
                        if (max_x < 0 || max_y < 0) continue;

                        std::uniform_int_distribution<> dist_x(0, max_x);
                        std::uniform_int_distribution<> dist_y(0, max_y);

                        moved.x = dist_x(gen);
                        moved.y = dist_y(gen);

                        auto nb = solution;
                        nb[idx] = moved;

                        // USE THE NEW RELAXED CHECK
                        if (is_valid_relaxed(moved, target_box_id, nb, idx)) {
                            add_neighbor(nb);
                            break;
                        }
                    }
                }
            }
        }
    }

    // LOOP 2: Shuffle within box
    for (const auto& [box_id, data] : box_data) {
        if (nbs.size() >= MAX_NEIGHBORS) break;
        // ... (existing logic) ...
        auto indices = data.rect_indices;
        std::shuffle(indices.begin(), indices.end(), gen);
        int max_shuffle = std::min(6, (int)indices.size());

        for (int i = 0; i < max_shuffle && nbs.size() < MAX_NEIGHBORS; i++) {
            int idx = indices[i];
            const auto& rect = solution[idx];

            int max_x = L - rect.get_actual_width();
            int max_y = L - rect.get_actual_height();
            if (max_x < 0 || max_y < 0) continue;

            std::uniform_int_distribution<> dist_x(0, max_x);
            std::uniform_int_distribution<> dist_y(0, max_y);

            for (int attempt = 0; attempt < 12 && nbs.size() < MAX_NEIGHBORS; attempt++) {
                for (int rot = 0; rot < 2; rot++) {
                    RectanglePlacement moved = rect;
                    if (rot == 1) moved.rotated = !moved.rotated;
                    // ... (existing random pos logic) ...
                    int rot_max_x = L - moved.get_actual_width();
                    int rot_max_y = L - moved.get_actual_height();
                    if (rot_max_x < 0 || rot_max_y < 0) continue;

                    std::uniform_int_distribution<> rot_dist_x(0, rot_max_x);
                    std::uniform_int_distribution<> rot_dist_y(0, rot_max_y);

                    moved.x = rot_dist_x(gen);
                    moved.y = rot_dist_y(gen);

                    auto nb = solution;
                    nb[idx] = moved;

                    // USE THE NEW RELAXED CHECK
                    if (is_valid_relaxed(moved, box_id, nb, idx)) {
                        add_neighbor(nb);
                        break;
                    }
                }
            }
        }
    }

    return nbs;
}

double RelaxedGeometryBasedNeighborhoodSolver::calculate_max_overlap_ratio(int T) {
    if (T <= 0) return 0.0;
    double max_ratio = 1.0 * (T / 1000.0);
    return std::min(1.0, max_ratio);
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_max_overlap_area(int T, int L) {
    if (T <= 0) return 0;
    int base_area = (L * L) / 4;
    return static_cast<int>(base_area * (T / 1000.0));
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_overlap_offset(int T, int rect_size) {
    if (T <= 0) return 0;
    int max_offset = rect_size;
    return static_cast<int>(max_offset * (T / 1000.0));
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_perturbation_range(int T) {
    if (T <= 0) return 0;
    return 5 + static_cast<int>(15 * (T / 1000.0));
}