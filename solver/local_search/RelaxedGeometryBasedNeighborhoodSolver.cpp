#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <set>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {
    return construct_overlapping_neighbors(problem, T);
}

// Helper: Calculate total overlap area in a solution
long long RelaxedGeometryBasedNeighborhoodSolver::calculate_total_overlap(
    const std::vector<RectanglePlacement>& solution) {

    long long total_overlap = 0;
    int n = solution.size();

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (solution[i].box_id != solution[j].box_id) continue;

            const auto& r1 = solution[i];
            const auto& r2 = solution[j];

            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + r1.get_actual_width(),
                                     r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + r1.get_actual_height(),
                                     r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                total_overlap += (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }
        }
    }

    return total_overlap;
}

// Helper: Generate random spread (used at high temperatures)
std::vector<RectanglePlacement>
RelaxedGeometryBasedNeighborhoodSolver::generate_random_spread(
    const std::vector<RectanglePlacement>& solution,
    int L, int T) {

    auto spread_solution = solution;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> pos_dist(0, L-1);

    double spread_factor = T / 1000.0;

    for (auto& rect : spread_solution) {
        rect.x = pos_dist(gen);
        rect.y = pos_dist(gen);

        if (rect.width != rect.height && (rand() % 100) < (T / 10)) {
            rect.rotated = !rect.rotated;
        }

        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x + w > L) rect.x = L - w;
        if (rect.y + h > L) rect.y = L - h;
    }

    return spread_solution;
}

// Helper: Generate swap move between boxes
std::vector<RectanglePlacement>
RelaxedGeometryBasedNeighborhoodSolver::generate_swap_move(
    const std::vector<RectanglePlacement>& solution,
    int L, int T) {

    auto swapped_solution = solution;
    int n = solution.size();
    if (n < 2) return swapped_solution;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> rect_dist(0, n-1);
    std::uniform_int_distribution<> pos_dist(0, L-1);

    int i = rect_dist(gen);
    int j = rect_dist(gen);
    while (i == j) j = rect_dist(gen);

    auto& rect1 = swapped_solution[i];
    auto& rect2 = swapped_solution[j];

    std::swap(rect1.box_id, rect2.box_id);

    rect1.x = pos_dist(gen);
    rect1.y = pos_dist(gen);
    rect2.x = pos_dist(gen);
    rect2.y = pos_dist(gen);

    if (rect1.width != rect1.height && (rand() % 100) < (T / 20)) {
        rect1.rotated = !rect1.rotated;
    }
    if (rect2.width != rect2.height && (rand() % 100) < (T / 20)) {
        rect2.rotated = !rect2.rotated;
    }

    int w1 = rect1.get_actual_width();
    int h1 = rect1.get_actual_height();
    int w2 = rect2.get_actual_width();
    int h2 = rect2.get_actual_height();

    if (rect1.x + w1 > L) rect1.x = L - w1;
    if (rect1.y + h1 > L) rect1.y = L - h1;
    if (rect2.x + w2 > L) rect2.x = L - w2;
    if (rect2.y + h2 > L) rect2.y = L - h2;

    return swapped_solution;
}

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_overlapping_neighbors(
    RectangleFittingProblem &problem, int T) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 500;

    std::cout << "\n=== RELAXED GEOMETRY SOLVER (T=" << T << ") ===" << std::endl;

    long long initial_overlap = calculate_total_overlap(solution);
    std::cout << "Initial overlap: " << initial_overlap << std::endl;

    // ======================================
    // KEY FIX: Temperature-based strategy
    // High T (>700): More exploration
    // Mid T (300-700): Balanced
    // Low T (<300): Heavy exploitation
    // ======================================

    double exploration_ratio = std::min(1.0, T / 700.0);  // 0.0 to 1.0
    double exploitation_ratio = 1.0 - exploration_ratio;

    std::cout << "Exploration ratio: " << exploration_ratio
              << ", Exploitation ratio: " << exploitation_ratio << std::endl;

    // PHASE 1: CALL PARENT'S GEOMETRY-BASED MOVES FIRST
    // These are GOOD moves that don't create overlaps
    auto geometry_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors(problem, T);

    // FIX: Use MORE geometry moves at LOW temperature
    int num_geometry_moves = std::min(
        (int)geometry_neighbors.size(),
        (int)(MAX_NEIGHBORS * (0.5 + 0.5 * exploitation_ratio))  // 50%-100% based on T
    );

    for (int i = 0; i < num_geometry_moves && nbs.size() < MAX_NEIGHBORS; i++) {
        nbs.push_back(geometry_neighbors[i]);
    }

    std::cout << "Added " << nbs.size() << " geometry-based moves (from "
              << geometry_neighbors.size() << " available)" << std::endl;

    // PHASE 2: EXPLORATION MOVES (only at high temperatures)
    int exploration_moves = 0;

    // A. Random spread moves (only at very high T)
    if (T > 800) {
        int num_spreads = std::min(2, MAX_NEIGHBORS - (int)nbs.size());
        for (int i = 0; i < num_spreads; i++) {
            nbs.push_back(generate_random_spread(solution, L, T));
            exploration_moves++;
        }
    }

    // B. Swap moves between boxes (at high-mid T)
    if (T > 500) {
        int num_swaps = std::min(1 + T / 500, MAX_NEIGHBORS - (int)nbs.size());
        for (int i = 0; i < num_swaps; i++) {
            nbs.push_back(generate_swap_move(solution, L, T));
            exploration_moves++;
        }
    }

    std::cout << "Added " << exploration_moves << " exploration moves" << std::endl;

    // PHASE 3: AGGRESSIVE OVERLAP RESOLUTION (at ALL temperatures if overlaps exist)
    if (initial_overlap > 0) {
        int resolution_moves = 0;
        int max_resolution = std::min(50, MAX_NEIGHBORS - (int)nbs.size());

        // Group rectangles by box
        std::unordered_map<int, std::vector<int>> box_rects;
        for (int i = 0; i < n; i++) {
            box_rects[solution[i].box_id].push_back(i);
        }

        // For each box, find and resolve overlaps
        for (const auto& [box_id, rect_indices] : box_rects) {
            if (resolution_moves >= max_resolution) break;

            // Find ALL pairs of overlapping rectangles
            std::vector<std::pair<int, int>> overlapping_pairs;
            for (size_t i = 0; i < rect_indices.size(); i++) {
                for (size_t j = i + 1; j < rect_indices.size(); j++) {
                    const auto& r1 = solution[rect_indices[i]];
                    const auto& r2 = solution[rect_indices[j]];
                    if (r1.collides(r2)) {
                        overlapping_pairs.push_back({rect_indices[i], rect_indices[j]});
                    }
                }
            }

            // Try to resolve each overlapping pair
            for (const auto& [idx1, idx2] : overlapping_pairs) {
                if (resolution_moves >= max_resolution) break;

                const auto& rect1 = solution[idx1];
                const auto& rect2 = solution[idx2];

                // Try moving rect1 away from rect2
                for (int attempt = 0; attempt < 5 && resolution_moves < max_resolution; attempt++) {
                    auto neighbor = solution;
                    auto& rect = neighbor[idx1];

                    // Calculate direction away from rect2
                    int center1_x = rect1.x + rect1.get_actual_width() / 2;
                    int center1_y = rect1.y + rect1.get_actual_height() / 2;
                    int center2_x = rect2.x + rect2.get_actual_width() / 2;
                    int center2_y = rect2.y + rect2.get_actual_height() / 2;

                    int dx = center1_x - center2_x;
                    int dy = center1_y - center2_y;

                    // Normalize and scale
                    double dist = sqrt(dx*dx + dy*dy);
                    if (dist > 0) {
                        dx = (int)(dx / dist * (10 + attempt * 5));
                        dy = (int)(dy / dist * (10 + attempt * 5));
                    }

                    int new_x = rect.x + dx;
                    int new_y = rect.y + dy;

                    // Clamp to bounds
                    int w = rect.get_actual_width();
                    int h = rect.get_actual_height();
                    new_x = std::max(0, std::min(L - w, new_x));
                    new_y = std::max(0, std::min(L - h, new_y));

                    rect.x = new_x;
                    rect.y = new_y;

                    // Check if this REDUCES overlap
                    long long new_overlap = calculate_total_overlap(neighbor);
                    if (new_overlap < initial_overlap) {
                        nbs.push_back(neighbor);
                        resolution_moves++;
                        break;
                    }

                    // Try rotation if applicable
                    if (rect.width != rect.height) {
                        rect.rotated = !rect.rotated;
                        w = rect.get_actual_width();
                        h = rect.get_actual_height();
                        if (rect.x + w > L) rect.x = L - w;
                        if (rect.y + h > L) rect.y = L - h;

                        new_overlap = calculate_total_overlap(neighbor);
                        if (new_overlap < initial_overlap) {
                            nbs.push_back(neighbor);
                            resolution_moves++;
                            break;
                        }
                    }
                }
            }
        }

        std::cout << "Added " << resolution_moves << " overlap resolution moves" << std::endl;
    }

    // PHASE 4: Small random perturbations (always useful)
    int perturbation_moves = 0;
    int max_perturbations = std::min(20, MAX_NEIGHBORS - (int)nbs.size());
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> rect_dist(0, n-1);

    for (int i = 0; i < max_perturbations; i++) {
        auto neighbor = solution;
        int rect_idx = rect_dist(gen);
        auto& rect = neighbor[rect_idx];

        // Small move (smaller at low T)
        int max_step = std::max(1, (int)(5.0 * exploration_ratio + 1));
        int dx = (rand() % (2 * max_step + 1)) - max_step;
        int dy = (rand() % (2 * max_step + 1)) - max_step;

        int new_x = rect.x + dx;
        int new_y = rect.y + dy;

        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        new_x = std::max(0, std::min(L - w, new_x));
        new_y = std::max(0, std::min(L - h, new_y));

        if (new_x != rect.x || new_y != rect.y) {
            rect.x = new_x;
            rect.y = new_y;

            // Only add if it doesn't make things worse
            long long new_overlap = calculate_total_overlap(neighbor);
            if (new_overlap <= initial_overlap * 1.1) {  // Allow 10% worse
                nbs.push_back(neighbor);
                perturbation_moves++;
            }
        }
    }

    std::cout << "Added " << perturbation_moves << " perturbation moves" << std::endl;
    std::cout << "Total neighbors generated: " << nbs.size() << std::endl;

    return nbs;
}