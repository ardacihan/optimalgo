// ============================================================================
// FILE: solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.cpp (UPDATED WITH DELTA)
// ============================================================================
#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <set>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {
    std::vector<NeighborMetadata> dummy_metadata;
    return construct_neighbors_with_metadata(problem, T, dummy_metadata);
}

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
    RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 1000;
    // PHASE 1: Geometry-based moves from parent class
    std::vector<NeighborMetadata> geometry_metadata;
    auto geometry_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors_with_metadata(
        problem, T, geometry_metadata);

    for (size_t i = 0; i < geometry_neighbors.size() && nbs.size() < MAX_NEIGHBORS; i++) {
        nbs.push_back(geometry_neighbors[i]);
        if (i < geometry_metadata.size()) {
            metadata.push_back(geometry_metadata[i]);
        } else {
            metadata.push_back(NeighborMetadata());
        }
    }

    // PHASE 2: Swap moves
    int swap_moves_remaining = MAX_NEIGHBORS - nbs.size();
    if (swap_moves_remaining > 0 && n >= 2) {
        std::vector<std::pair<int, int>> potential_swaps;

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                potential_swaps.push_back({i, j});
            }
        }

        int max_swaps = std::min(swap_moves_remaining, 50);
        int swaps_generated = 0;

        for (const auto& [i, j] : potential_swaps) {
            if (swaps_generated >= max_swaps) break;

            auto neighbor = solution;
            std::swap(neighbor[i].box_id, neighbor[j].box_id);

            nbs.push_back(neighbor);
            metadata.push_back(NeighborMetadata());
            swaps_generated++;
        }
    }

    // PHASE 3: Assign new box_id to overlapping rectangles (when T is low)
    if (T < 500) { // Low temperature threshold
        std::vector<int> overlapping_rects;

        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (solution[i].box_id == solution[j].box_id && solution[i].collides(solution[j])) {
                    overlapping_rects.push_back(i);
                    overlapping_rects.push_back(j);
                }
            }
        }

        std::sort(overlapping_rects.begin(), overlapping_rects.end());
        overlapping_rects.erase(std::unique(overlapping_rects.begin(), overlapping_rects.end()), overlapping_rects.end());

        int moves_remaining = MAX_NEIGHBORS - nbs.size();
        int moves_generated = 0;

        for (int rect_idx : overlapping_rects) {
            if (moves_generated >= moves_remaining) break;

            int max_box_id = 0;
            for (const auto& r : solution) {
                max_box_id = std::max(max_box_id, r.box_id);
            }

            auto neighbor = solution;
            neighbor[rect_idx].box_id = max_box_id + 1;
            neighbor[rect_idx].x = 0;
            neighbor[rect_idx].y = 0;

            nbs.push_back(neighbor);
            metadata.push_back(NeighborMetadata(rect_idx));
            moves_generated++;
        }
    }

    return nbs;
}