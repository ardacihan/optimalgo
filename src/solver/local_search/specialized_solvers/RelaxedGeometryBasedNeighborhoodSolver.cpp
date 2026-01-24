#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <random>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <queue>
#include <functional>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {

    const auto& solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();
    long long box_capacity = (long long)L * L;

    const int MAX_NEIGHBORS = 80;
    std::vector<std::vector<RectanglePlacement>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);
    if (n == 0) return neighbors;


    if(T >= 200) {
        add_exploration_moves(solution, L, box_capacity, neighbors, MAX_NEIGHBORS / 3);
    }


    if (T < 500) {
        add_fixing_moves(solution,L,box_capacity,neighbors,MAX_NEIGHBORS/ 3);
    }

    auto geo_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors(
        problem, T);

    for (size_t i = 0; i < geo_neighbors.size() && neighbors.size() < MAX_NEIGHBORS; i++) {
        neighbors.push_back(geo_neighbors[i]);
    }



    return neighbors;
}

void RelaxedGeometryBasedNeighborhoodSolver::add_exploration_moves(
    const std::vector<RectanglePlacement>& solution,
    int L,
    long long box_capacity,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    int max_neighbors) {

    int n = solution.size();
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idx_dist(0, n-1);
    std::uniform_int_distribution<int> bool_dist(0, 1);

    int exploration_budget = max_neighbors - neighbors.size();

    // Calculate current box utilizations
    std::unordered_map<int, long long> box_area;
    for (const auto& rect : solution) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        box_area[rect.box_id] += (long long)w * h;
    }

    int move_count = exploration_budget /2;
    for (int attempt = 0; attempt < move_count * 3 && neighbors.size() < max_neighbors; attempt++) {
        int idx = idx_dist(rng);
        auto neighbor = solution;

        int w = neighbor[idx].get_actual_width();
        int h = neighbor[idx].get_actual_height();
        long long rect_area = (long long)w * h;
        int current_box = neighbor[idx].box_id;

        // Find target boxes that have enough capacity
        std::vector<int> candidate_boxes;

        // Check existing boxes (excluding current box)
        for (const auto& [box_id, area] : box_area) {
            if (box_id != current_box && area + rect_area <= box_capacity) {
                candidate_boxes.push_back(box_id);
            }
        }

        // Always consider creating a new box
        int max_box_id = 0;
        for (const auto& r : neighbor) {
            max_box_id = std::max(max_box_id, r.box_id);
        }
        candidate_boxes.push_back(max_box_id + 1); // New box always has capacity

        if (candidate_boxes.empty()) continue;

        std::uniform_int_distribution<int> box_dist(0, candidate_boxes.size() - 1);
        int target_box = candidate_boxes[box_dist(rng)];

        neighbor[idx].box_id = target_box;

        // Position in top-left corner of new box
        if (target_box > max_box_id) {
            neighbor[idx].x = 0;
            neighbor[idx].y = 0;
        } else {
            // For existing box, try to find a position (allow overlaps for now)
            std::uniform_int_distribution<int> x_dist(0, L - w);
            std::uniform_int_distribution<int> y_dist(0, L - h);
            neighbor[idx].x = x_dist(rng);
            neighbor[idx].y = y_dist(rng);
        }

        neighbors.push_back(neighbor);
    }

    int swap_count = exploration_budget /2;
    for (int attempt = 0; attempt < swap_count * 3 && neighbors.size() < max_neighbors; attempt++) {
        if (n < 2) break;

        int i = idx_dist(rng);
        int j = idx_dist(rng);
        if (i == j || solution[i].box_id == solution[j].box_id) continue;

        auto neighbor = solution;

        int w1 = neighbor[i].get_actual_width();
        int h1 = neighbor[i].get_actual_height();
        long long area1 = (long long)w1 * h1;
        int box1 = neighbor[i].box_id;

        int w2 = neighbor[j].get_actual_width();
        int h2 = neighbor[j].get_actual_height();
        long long area2 = (long long)w2 * h2;
        int box2 = neighbor[j].box_id;

        // Check capacity constraints for swap
        long long current_area1 = box_area[box1];
        long long current_area2 = box_area[box2];

        // After swap: box1 gets area2, loses area1
        // After swap: box2 gets area1, loses area2
        long long new_area_box1 = current_area1 - area1 + area2;
        long long new_area_box2 = current_area2 - area2 + area1;

        if (new_area_box1 <= box_capacity && new_area_box2 <= box_capacity) {
            std::swap(neighbor[i].box_id, neighbor[j].box_id);

            // Reset positions to avoid immediate overlaps
            neighbor[i].x = 0;
            neighbor[i].y = 0;
            neighbor[j].x = 0;
            neighbor[j].y = 0;

            neighbors.push_back(neighbor);
        }
    }
}

void RelaxedGeometryBasedNeighborhoodSolver::add_fixing_moves(
    const std::vector<RectanglePlacement>& solution,
    int L,
    long long box_capacity,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    int max_neighbors) {

    int n = solution.size();
    if (n < 2) return;

    // Calculate current box utilizations
    std::unordered_map<int, long long> box_area;
    for (const auto& rect : solution) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        box_area[rect.box_id] += (long long)w * h;
    }

    // Group by box_id to find overlaps
    std::unordered_map<int, std::vector<int>> box_to_rects;
    for (int i = 0; i < n; i++) {
        box_to_rects[solution[i].box_id].push_back(i);
    }

    // Find overlapping rectangles
    std::vector<std::pair<int, int>> overlapping_pairs;
    for (const auto& [box_id, indices] : box_to_rects) {
        for (size_t i = 0; i < indices.size(); i++) {
            for (size_t j = i + 1; j < indices.size(); j++) {
                int idx1 = indices[i];
                int idx2 = indices[j];
                if (solution[idx1].collides(solution[idx2])) {
                    overlapping_pairs.emplace_back(idx1, idx2);
                }
            }
        }
    }

    if (overlapping_pairs.empty()) return;

    int move_count = 0;

    for (const auto& [idx1, idx2] : overlapping_pairs) {
        if (move_count >= max_neighbors) break;

        // Find the smaller rectangle
        int area1 = solution[idx1].width * solution[idx1].height;
        int area2 = solution[idx2].width * solution[idx2].height;
        int idx_to_move = (area1 < area2) ? idx1 : idx2;

        auto neighbor = solution;

        // Find max box_id and create new one
        int max_box_id = 0;
        for (const auto& r : neighbor) {
            max_box_id = std::max(max_box_id, r.box_id);
        }

        neighbor[idx_to_move].box_id = max_box_id + 1;
        neighbor[idx_to_move].x = 0;
        neighbor[idx_to_move].y = 0;

        neighbors.push_back(neighbor);
        move_count++;
    }

}

bool RelaxedGeometryBasedNeighborhoodSolver::is_acceptable(
    const std::vector<RectanglePlacement>& neighbor,
    int L,
    double overlap_tolerance) {

    // Check bounds
    for (const auto& rect : neighbor) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x < 0 || rect.y < 0 || rect.x + w > L || rect.y + h > L) {
            return false;
        }
    }

    // For the relaxed solver, we accept all solutions during search
    // Overlaps will be fixed by the fixing moves
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

    // Evaluate all neighbors
    std::vector<std::tuple<int, int, bool>> neighbor_scores; // (objective, index, has_overlaps)
    for (size_t i = 0; i < neighbors.size(); i++) {
        int obj = problem.objective(neighbors[i]);
        bool overlaps = has_overlaps(neighbors[i]);
        neighbor_scores.emplace_back(obj, i, overlaps);
    }

    // Sort by: 1. No overlaps, 2. Higher objective
    std::sort(neighbor_scores.begin(), neighbor_scores.end(),
        [](const auto& a, const auto& b) {
            bool no_overlap_a = !std::get<2>(a);
            bool no_overlap_b = !std::get<2>(b);

            if (no_overlap_a && !no_overlap_b) return true;
            if (!no_overlap_a && no_overlap_b) return false;

            // Both have same overlap status, compare objective
            return std::get<0>(a) > std::get<0>(b);
        });

    // Try to accept the best neighbor
    for (const auto& [obj, idx, has_overlap] : neighbor_scores) {
        // At high temperatures, we might accept overlapping solutions
        // At low temperatures, prefer non-overlapping solutions
        double accept_probability = has_overlap ? (T / 1000.0) : 1.0;

        static std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        if (obj >= current_obj || dist(rng) < accept_probability) {
            problem.set_current_solution(neighbors[idx]);
            return neighbors[idx];
        }
    }

    // If no neighbor accepted, return current solution
    return current_solution;
}
