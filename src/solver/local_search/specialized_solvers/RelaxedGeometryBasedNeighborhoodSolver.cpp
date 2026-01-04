#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <random>
#include <algorithm>
#include <unordered_map>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {

    const auto& solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 100;
    std::vector<std::vector<RectanglePlacement>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);
    if (n == 0) return neighbors;

    const double T_MAX = 1000.0;
    double overlap_tolerance = std::max(0.0, std::min(1.0, T / T_MAX));

    auto geo_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors(
        problem, T);

    for (size_t i = 0; i < geo_neighbors.size() && neighbors.size() < MAX_NEIGHBORS; i++) {
        if (is_acceptable(geo_neighbors[i], L, overlap_tolerance)) {
            neighbors.push_back(geo_neighbors[i]);
        }
    }

    if (overlap_tolerance > 0.1) {
        add_exploration_moves(solution, L, overlap_tolerance, neighbors, MAX_NEIGHBORS);
    }

    return neighbors;
}

void RelaxedGeometryBasedNeighborhoodSolver::add_exploration_moves(
    const std::vector<RectanglePlacement>& solution,
    int L,
    double overlap_tolerance,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    int max_neighbors) {

    int n = solution.size();
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idx_dist(0, n-1);

    int exploration_budget = (int)((max_neighbors - neighbors.size()) * overlap_tolerance);

    int reposition_count = exploration_budget * 0.4;
    for (int attempt = 0; attempt < reposition_count * 3 && neighbors.size() < max_neighbors; attempt++) {
        int idx = idx_dist(rng);
        auto neighbor = solution;

        int w = neighbor[idx].get_actual_width();
        int h = neighbor[idx].get_actual_height();

        if (w > L || h > L) continue;

        std::uniform_int_distribution<int> x_dist(0, L - w);
        std::uniform_int_distribution<int> y_dist(0, L - h);

        neighbor[idx].x = x_dist(rng);
        neighbor[idx].y = y_dist(rng);

        if (is_acceptable(neighbor, L, overlap_tolerance)) {
            neighbors.push_back(neighbor);
        }
    }

    int swap_count = exploration_budget * 0.4;
    for (int attempt = 0; attempt < swap_count * 3 && neighbors.size() < max_neighbors; attempt++) {
        if (n < 2) break;

        int i = idx_dist(rng);
        int j = idx_dist(rng);
        if (i == j || solution[i].box_id == solution[j].box_id) continue;

        auto neighbor = solution;
        std::swap(neighbor[i].box_id, neighbor[j].box_id);

        if (is_acceptable(neighbor, L, overlap_tolerance)) {
            neighbors.push_back(neighbor);
        }
    }

    int overlap_move_count = exploration_budget * 0.2;
    for (int attempt = 0; attempt < overlap_move_count * 3 && neighbors.size() < max_neighbors; attempt++) {
        if (n < 2) break;

        int idx = idx_dist(rng);
        auto neighbor = solution;

        std::vector<int> candidates;
        for (int i = 0; i < n; i++) {
            if (i != idx && solution[i].box_id == solution[idx].box_id) {
                candidates.push_back(i);
            }
        }

        if (candidates.empty()) continue;

        std::uniform_int_distribution<int> cand_dist(0, candidates.size() - 1);
        int target_idx = candidates[cand_dist(rng)];

        int w = neighbor[idx].get_actual_width();
        int h = neighbor[idx].get_actual_height();

        if (w > L || h > L) continue;

        const auto& target = solution[target_idx];
        int target_w = target.get_actual_width();
        int target_h = target.get_actual_height();

        int min_x = std::max(0, target.x - w + 1);
        int max_x = std::min(L - w, target.x + target_w - 1);
        int min_y = std::max(0, target.y - h + 1);
        int max_y = std::min(L - h, target.y + target_h - 1);

        if (min_x > max_x || min_y > max_y) continue;

        std::uniform_int_distribution<int> x_dist(min_x, max_x);
        std::uniform_int_distribution<int> y_dist(min_y, max_y);

        neighbor[idx].x = x_dist(rng);
        neighbor[idx].y = y_dist(rng);

        if (is_acceptable(neighbor, L, overlap_tolerance)) {
            neighbors.push_back(neighbor);
        }
    }
}

bool RelaxedGeometryBasedNeighborhoodSolver::is_acceptable(
    const std::vector<RectanglePlacement>& neighbor,
    int L,
    double overlap_tolerance) {

    for (const auto& rect : neighbor) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x < 0 || rect.y < 0 || rect.x + w > L || rect.y + h > L) {
            return false;
        }
    }

    if (overlap_tolerance >= 0.99) {
        return true;
    }

    if (overlap_tolerance <= 0.01) {
        return !has_overlaps(neighbor);
    }

    double max_allowed_overlap_ratio = overlap_tolerance;

    int n = neighbor.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (neighbor[i].box_id == neighbor[j].box_id) {
                long long overlap_area = neighbor[i].getOverlapArea(neighbor[j]);
                if (overlap_area > 0) {
                    int w1 = neighbor[i].get_actual_width();
                    int h1 = neighbor[i].get_actual_height();
                    int w2 = neighbor[j].get_actual_width();
                    int h2 = neighbor[j].get_actual_height();

                    long long area1 = (long long)w1 * h1;
                    long long area2 = (long long)w2 * h2;

                    long long smaller_area = std::min(area1, area2);

                    double overlap_ratio = static_cast<double>(overlap_area) / smaller_area;

                    if (overlap_ratio > max_allowed_overlap_ratio) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

bool RelaxedGeometryBasedNeighborhoodSolver::has_overlaps(
    const std::vector<RectanglePlacement>& solution) {

    int n = solution.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (solution[i].box_id == solution[j].box_id &&
                solution[i].collides(solution[j])) {
                return true;
            }
        }
    }
    return false;
}

std::vector<RectanglePlacement> RelaxedGeometryBasedNeighborhoodSolver::solve_one_step(
    RectangleFittingProblem &problem, int T) {

    auto neighbors = construct_neighbors(problem, T);

    auto current_solution = problem.get_current_solution();
    int current_obj = problem.objective(current_solution);

    if (neighbors.empty()) {
        return current_solution;
    }

    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idx_dist(0, neighbors.size() - 1);
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    int random_idx = idx_dist(rng);
    auto candidate_solution = neighbors[random_idx];
    int candidate_obj = problem.objective(candidate_solution);

    bool accept = false;

    if (candidate_obj >= current_obj) {
        accept = true;
    } else {
        int delta = current_obj - candidate_obj;
        double acceptance_prob = std::exp(-delta / (double)T);

        if (prob_dist(rng) < acceptance_prob) {
            accept = true;
        }
    }

    if (accept) {
        problem.set_current_solution(candidate_solution);
        return candidate_solution;
    } else {
        return current_solution;
    }
}