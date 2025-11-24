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

    int MAX_NEIGHBORS = 200;
    nbs.reserve(MAX_NEIGHBORS);

    std::random_device rd;
    std::mt19937 gen(rd());

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
        it->second.total_area += solution[i].get_actual_width() * solution[i].get_actual_height();
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

    auto collides_in_box = [&](const RectanglePlacement& r, int box_id,
                                const std::vector<RectanglePlacement>& sol, int skip_idx = -1) {
        for (int i = 0; i < (int)sol.size(); i++) {
            if (i == skip_idx) continue;
            if (sol[i].box_id != box_id) continue;

            const auto& other = sol[i];
            if (!(r.x >= other.x + other.get_actual_width() ||
                  r.x + r.get_actual_width() <= other.x ||
                  r.y >= other.y + other.get_actual_height() ||
                  r.y + r.get_actual_height() <= other.y)) {
                return true;
            }
        }
        return false;
    };

    for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
        int sparse_box_id = box_utilizations[i].first;
        double sparse_util = box_utilizations[i].second;

        if (sparse_util > 0.7) break;

        auto sparse_it = box_data.find(sparse_box_id);
        if (sparse_it == box_data.end()) continue;
        const auto& sparse_data = sparse_it->second;

        auto indices = sparse_data.rect_indices;
        std::shuffle(indices.begin(), indices.end(), gen);

        int max_rects_to_move = std::min(5, (int)indices.size());

        for (int rect_num = 0; rect_num < max_rects_to_move && nbs.size() < MAX_NEIGHBORS; rect_num++) {
            int idx = indices[rect_num];
            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            for (size_t j = box_utilizations.size() - 1; j > i && nbs.size() < MAX_NEIGHBORS; j--) {
                int target_box_id = box_utilizations[j].first;
                if (target_box_id == sparse_box_id) continue;

                auto target_it = box_data.find(target_box_id);
                if (target_it == box_data.end()) continue;
                const auto& target_data = target_it->second;

                long long target_area = target_data.total_area;
                long long box_capacity = (long long)L * L;

                if (target_area + rect_area > box_capacity * 0.85) continue;

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

                        if (!collides_in_box(moved, target_box_id, nb, idx)) {
                            add_neighbor(nb);
                            break;
                        }
                    }
                }
            }
        }
    }

    for (const auto& [box_id, data] : box_data) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

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

                    int rot_max_x = L - moved.get_actual_width();
                    int rot_max_y = L - moved.get_actual_height();
                    if (rot_max_x < 0 || rot_max_y < 0) continue;

                    std::uniform_int_distribution<> rot_dist_x(0, rot_max_x);
                    std::uniform_int_distribution<> rot_dist_y(0, rot_max_y);

                    moved.x = rot_dist_x(gen);
                    moved.y = rot_dist_y(gen);

                    auto nb = solution;
                    nb[idx] = moved;

                    if (!collides_in_box(moved, box_id, nb, idx)) {
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