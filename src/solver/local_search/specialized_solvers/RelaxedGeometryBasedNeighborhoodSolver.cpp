// ============================================================================
// FILE: solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.cpp (SIMPLIFIED)
// ============================================================================
#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <random>
#include <algorithm>
#include <unordered_map>
#include <iostream>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {

    const auto& solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 500;
    std::vector<std::vector<RectanglePlacement>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);
    if (n == 0) return neighbors;

    // Calculate overlap tolerance: T=1000 -> 1.0, T=0 -> 0.0
    const double T_MAX = 1000.0;
    double overlap_tolerance = std::max(0.0, std::min(1.0, T / T_MAX));

    // 1. Always get geometry-based moves (they're good quality)
    auto geo_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors(
        problem, T);

    // Add all geometry moves that respect overlap tolerance
    for (size_t i = 0; i < geo_neighbors.size() && neighbors.size() < MAX_NEIGHBORS; i++) {
        if (is_acceptable(geo_neighbors[i], L, overlap_tolerance)) {
            neighbors.push_back(geo_neighbors[i]);
        }
    }

    int geo_count = neighbors.size();

    // 2. Add aggressive exploration moves at high temperature
    if (overlap_tolerance > 0.3) { // Only when T > 600
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

    // How many exploration moves based on temperature
    int exploration_budget = (int)((max_neighbors - neighbors.size()) * overlap_tolerance);

    // 1. Random repositioning within bounds (allows overlaps at high T)
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

    // 2. Box swaps (move rectangles between boxes)
    int swap_count = exploration_budget * 0.3;
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

    // 3. Cluster breaking: move rectangles from crowded boxes to new boxes
    int cluster_count = exploration_budget * 0.3;
    if (cluster_count > 0) {
        std::unordered_map<int, int> box_counts;
        for (const auto& rect : solution) {
            box_counts[rect.box_id]++;
        }

        int max_box = 0;
        for (const auto& rect : solution) {
            max_box = std::max(max_box, rect.box_id);
        }

        // Find most crowded box
        int crowded_box = -1;
        int max_count = 0;
        for (const auto& [box_id, count] : box_counts) {
            if (count > max_count) {
                max_count = count;
                crowded_box = box_id;
            }
        }

        if (crowded_box != -1 && max_count > 2) {
            for (int idx = 0; idx < n && neighbors.size() < max_neighbors; idx++) {
                if (solution[idx].box_id == crowded_box) {
                    auto neighbor = solution;
                    neighbor[idx].box_id = max_box + 1;
                    neighbor[idx].x = 0;
                    neighbor[idx].y = 0;

                    if (is_acceptable(neighbor, L, overlap_tolerance)) {
                        neighbors.push_back(neighbor);
                    }
                }
            }
        }
    }
}

bool RelaxedGeometryBasedNeighborhoodSolver::is_acceptable(
    const std::vector<RectanglePlacement>& neighbor,
    int L,
    double overlap_tolerance) {

    // ALWAYS check bounds - never allow out-of-bounds
    for (const auto& rect : neighbor) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x < 0 || rect.y < 0 || rect.x + w > L || rect.y + h > L) {
            return false;
        }
    }

    // High temperature: accept all in-bounds moves (overlaps OK)
    if (overlap_tolerance >= 0.99) {
        return true;
    }

    // Low temperature: reject any overlaps
    if (overlap_tolerance <= 0.01) {
        return !has_overlaps(neighbor);
    }

    // Medium temperature: probabilistic acceptance
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    if (has_overlaps(neighbor)) {
        return dist(rng) < overlap_tolerance;
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


// Add this method to your RelaxedGeometryBasedNeighborhoodSolver class

std::vector<RectanglePlacement> RelaxedGeometryBasedNeighborhoodSolver::solve_one_step(
    RectangleFittingProblem &problem, int T) {

    // Get all neighbors based on current temperature
    auto neighbors = construct_neighbors(problem, T);

    // Current solution and its objective
    auto current_solution = problem.get_current_solution();
    int current_obj = problem.objective(current_solution);

    // If no neighbors, return current solution
    if (neighbors.empty()) {
        return current_solution;
    }

    // SIMULATED ANNEALING: Pick a random neighbor and decide whether to accept
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idx_dist(0, neighbors.size() - 1);
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);

    // Select a random neighbor (not the best one!)
    int random_idx = idx_dist(rng);
    auto candidate_solution = neighbors[random_idx];
    int candidate_obj = problem.objective(candidate_solution);

    // Decide whether to accept this candidate
    bool accept = false;

    if (candidate_obj >= current_obj) {
        // Always accept improvements (or equal solutions)
        accept = true;
    } else {
        // For worse solutions, accept with probability based on temperature
        // Higher T = more willing to accept worse solutions

        int delta = current_obj - candidate_obj; // Positive when worse

        // Metropolis criterion: P(accept) = exp(-delta / T)
        // Since we're maximizing, larger delta (worse move) = lower probability
        double acceptance_prob = std::exp(-delta / (double)T);

        // Accept if random value is below acceptance probability
        if (prob_dist(rng) < acceptance_prob) {
            accept = true;
        }
    }

    if (accept) {
        problem.set_current_solution(candidate_solution);
        return candidate_solution;
    } else {
        // Keep current solution
        return current_solution;
    }
}