// ============================================================================
// FILE: solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.cpp (TRULY DIFFERENT)
// ============================================================================
#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <random>
#include <algorithm>
#include <unordered_map>
#include <set>
#include <iostream>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {
    std::vector<NeighborMetadata> metadata;
    return construct_neighbors_with_metadata(problem, T, metadata);
}

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
    RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) {

    const auto& solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 800;

    std::vector<std::vector<RectanglePlacement>> neighbors;
    neighbors.reserve(MAX_NEIGHBORS);
    metadata.clear();

    if (n == 0) return neighbors;

    std::cout << "[RelaxedSolver] T = " << T << " | ";

    // AGGRESSIVE TEMPERATURE-BASED BEHAVIOR:
    // T >= 1000: Very aggressive - allow overlaps, random moves
    // 400 <= T < 1000: Moderately aggressive - swap-focused
    // T < 400: Conservative - mostly geometry with some swaps

    if (T >= 1000) {
        std::cout << "VERY AGGRESSIVE mode (T >= 1000)" << std::endl;
        // VERY AGGRESSIVE: Random moves, overlaps allowed
        generate_aggressive_moves(solution, L, neighbors, metadata, MAX_NEIGHBORS, T);
    }
    else if (T >= 400) {
        std::cout << "AGGRESSIVE mode (400 <= T < 1000)" << std::endl;
        // AGGRESSIVE: Swap-focused
        generate_swap_focused_moves(problem, solution, L, neighbors, metadata, MAX_NEIGHBORS, T);
    }
    else {
        std::cout << "CONSERVATIVE mode (T < 400)" << std::endl;
        // CONSERVATIVE: Geometry with some smart swaps
        generate_conservative_moves(problem, solution, L, neighbors, metadata, MAX_NEIGHBORS, T);
    }

    std::cout << "[RelaxedSolver] Total neighbors: " << neighbors.size() << std::endl;
    return neighbors;
}

void RelaxedGeometryBasedNeighborhoodSolver::generate_aggressive_moves(
    const std::vector<RectanglePlacement>& solution,
    int L,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    std::vector<NeighborMetadata>& metadata,
    int max_neighbors,
    int T) {

    int n = solution.size();
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idx_dist(0, n-1);
    std::uniform_int_distribution<int> pos_dist(0, L-1);
    std::uniform_int_distribution<int> bool_dist(0, 1);

    // 1. Random position moves (overlaps allowed!)
    for (int i = 0; i < n && neighbors.size() < max_neighbors * 0.4; i++) {
        auto neighbor = solution;
        int idx = idx_dist(rng);

        neighbor[idx].x = pos_dist(rng);
        neighbor[idx].y = pos_dist(rng);

        // Random rotation (if not square)
        if (neighbor[idx].width != neighbor[idx].height && bool_dist(rng)) {
            neighbor[idx].rotated = !neighbor[idx].rotated;
        }

        // Random box change (30% chance)
        if (bool_dist(rng) && bool_dist(rng) && bool_dist(rng)) {
            std::uniform_int_distribution<int> box_dist(0, 10);
            neighbor[idx].box_id = box_dist(rng);
        }

        neighbors.push_back(neighbor);
        metadata.push_back(NeighborMetadata(idx));
    }

    // 2. Random swaps (lots of them)
    while (neighbors.size() < max_neighbors * 0.8 && n >= 2) {
        int i = idx_dist(rng);
        int j = idx_dist(rng);
        if (i == j) continue;

        auto neighbor = solution;

        // Randomly decide what to swap
        if (bool_dist(rng)) std::swap(neighbor[i].box_id, neighbor[j].box_id);
        if (bool_dist(rng)) std::swap(neighbor[i].x, neighbor[j].x);
        if (bool_dist(rng)) std::swap(neighbor[i].y, neighbor[j].y);
        if (bool_dist(rng) && solution[i].width != solution[i].height &&
            solution[j].width != solution[j].height) {
            std::swap(neighbor[i].rotated, neighbor[j].rotated);
        }

        neighbors.push_back(neighbor);
        metadata.push_back(NeighborMetadata());
    }

    // 3. Cluster breaking: Move rectangles from crowded boxes to new boxes
    if (neighbors.size() < max_neighbors) {
        std::unordered_map<int, int> box_counts;
        for (const auto& rect : solution) {
            box_counts[rect.box_id]++;
        }

        // Find crowded boxes (more than average)
        std::vector<int> crowded_boxes;
        double avg = n / (double)box_counts.size();
        for (const auto& [box_id, count] : box_counts) {
            if (count > avg * 1.5) {
                crowded_boxes.push_back(box_id);
            }
        }

        for (int box_id : crowded_boxes) {
            for (int idx = 0; idx < n && neighbors.size() < max_neighbors; idx++) {
                if (solution[idx].box_id == box_id) {
                    auto neighbor = solution;
                    neighbor[idx].box_id = box_counts.size(); // New box
                    neighbor[idx].x = 0;
                    neighbor[idx].y = 0;

                    neighbors.push_back(neighbor);
                    metadata.push_back(NeighborMetadata(idx));
                    break;
                }
            }
        }
    }
}

void RelaxedGeometryBasedNeighborhoodSolver::generate_swap_focused_moves(
    RectangleFittingProblem &problem,
    const std::vector<RectanglePlacement>& solution,
    int L,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    std::vector<NeighborMetadata>& metadata,
    int max_neighbors,
    int T) {

    int n = solution.size();
    static std::mt19937 rng(std::random_device{}());

    // 1. Start with SOME geometry moves (25%)
    std::vector<NeighborMetadata> geo_metadata;
    auto geo_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
        problem, T, geo_metadata);

    int geo_to_take = std::min((int)geo_neighbors.size(), max_neighbors / 4);
    for (size_t i = 0; i < geo_to_take; i++) {
        neighbors.push_back(geo_neighbors[i]);
        if (i < geo_metadata.size()) {
            metadata.push_back(geo_metadata[i]);
        } else {
            metadata.push_back(NeighborMetadata());
        }
    }
    std::cout << "[RelaxedSolver]   Geometry moves: " << geo_to_take << std::endl;

    // 2. Smart swaps based on rectangle characteristics
    std::uniform_int_distribution<int> dist(0, n-1);

    // Swap by size similarity
    for (int s = 0; s < max_neighbors / 4 && neighbors.size() < max_neighbors * 0.5; s++) {
        int i = dist(rng);
        int j = dist(rng);
        if (i == j) continue;

        // Check if rectangles have similar area
        int area_i = solution[i].width * solution[i].height;
        int area_j = solution[j].width * solution[j].height;
        double ratio = std::min(area_i, area_j) / (double)std::max(area_i, area_j);

        if (ratio > 0.7) { // Similar size
            auto neighbor = solution;
            std::swap(neighbor[i].box_id, neighbor[j].box_id);
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata());
        }
    }

    // 3. Cross-box position exchanges
    for (int i = 0; i < n && neighbors.size() < max_neighbors * 0.75; i++) {
        for (int j = i + 1; j < n && neighbors.size() < max_neighbors * 0.75; j++) {
            if (solution[i].box_id == solution[j].box_id) continue;

            // Check if positions would be valid in swapped boxes
            const auto& r1 = solution[i];
            const auto& r2 = solution[j];

            bool r1_fits_in_r2_box = (r1.x + r1.get_actual_width() <= L && r1.y + r1.get_actual_height() <= L);
            bool r2_fits_in_r1_box = (r2.x + r2.get_actual_width() <= L && r2.y + r2.get_actual_height() <= L);

            if (r1_fits_in_r2_box && r2_fits_in_r1_box) {
                auto neighbor = solution;
                neighbor[i].box_id = r2.box_id;
                neighbor[j].box_id = r1.box_id;

                neighbors.push_back(neighbor);
                metadata.push_back(NeighborMetadata());
                break; // One per i
            }
        }
    }

    // 4. Rotation swaps for non-squares
    for (int i = 0; i < n && neighbors.size() < max_neighbors; i++) {
        if (solution[i].width == solution[i].height) continue;

        auto neighbor = solution;
        neighbor[i].rotated = !solution[i].rotated;

        // Check if still fits
        if (neighbor[i].x + neighbor[i].get_actual_width() <= L &&
            neighbor[i].y + neighbor[i].get_actual_height() <= L) {
            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(i));
        }
    }
}

void RelaxedGeometryBasedNeighborhoodSolver::generate_conservative_moves(
    RectangleFittingProblem &problem,
    const std::vector<RectanglePlacement>& solution,
    int L,
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    std::vector<NeighborMetadata>& metadata,
    int max_neighbors,
    int T) {

    int n = solution.size();

    // 1. Mostly geometry moves (70%)
    std::vector<NeighborMetadata> geo_metadata;
    auto geo_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
        problem, T, geo_metadata);

    int geo_to_take = std::min((int)geo_neighbors.size(), max_neighbors * 7 / 10);
    for (size_t i = 0; i < geo_to_take; i++) {
        neighbors.push_back(geo_neighbors[i]);
        if (i < geo_metadata.size()) {
            metadata.push_back(geo_metadata[i]);
        } else {
            metadata.push_back(NeighborMetadata());
        }
    }
    std::cout << "[RelaxedSolver]   Geometry moves: " << geo_to_take << std::endl;

    // 2. Fix overlapping rectangles
    if (neighbors.size() < max_neighbors * 0.9) {
        std::vector<int> overlapping;

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (solution[i].box_id == solution[j].box_id &&
                    solution[i].collides(solution[j])) {
                    overlapping.push_back(i);
                    overlapping.push_back(j);
                }
            }
        }

        std::sort(overlapping.begin(), overlapping.end());
        overlapping.erase(std::unique(overlapping.begin(), overlapping.end()), overlapping.end());

        for (int idx : overlapping) {
            if (neighbors.size() >= max_neighbors) break;

            // Find max box ID
            int max_box = 0;
            for (const auto& r : solution) {
                max_box = std::max(max_box, r.box_id);
            }

            auto neighbor = solution;
            neighbor[idx].box_id = max_box + 1;
            neighbor[idx].x = 0;
            neighbor[idx].y = 0;

            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata(idx));
        }
    }

    // 3. Strategic swaps only
    if (neighbors.size() < max_neighbors) {
        // Swap rectangles that are isolated (not touching others)
        std::vector<int> isolated_indices;

        for (int i = 0; i < n; i++) {
            bool touches_any = false;
            const auto& r1 = solution[i];

            for (int j = 0; j < n; j++) {
                if (i == j) continue;
                const auto& r2 = solution[j];
                if (r1.box_id == r2.box_id && r1.adjacent(r2)) {
                    touches_any = true;
                    break;
                }
            }

            if (!touches_any) {
                isolated_indices.push_back(i);
            }
        }

        // Swap isolated rectangles with each other
        for (size_t i = 0; i + 1 < isolated_indices.size() && neighbors.size() < max_neighbors; i += 2) {
            int idx1 = isolated_indices[i];
            int idx2 = isolated_indices[i + 1];

            auto neighbor = solution;
            std::swap(neighbor[idx1].box_id, neighbor[idx2].box_id);

            neighbors.push_back(neighbor);
            metadata.push_back(NeighborMetadata());
        }
    }
}