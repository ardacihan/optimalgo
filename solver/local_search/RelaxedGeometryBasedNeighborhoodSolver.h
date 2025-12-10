
// ============================================================================
// FILE: solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.h (UPDATED)
// ============================================================================
#ifndef RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H
#define RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>

class RelaxedGeometryBasedNeighborhoodSolver : public GeometryBasedNeighborhoodSolver {
public:
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;
    
    std::vector<std::vector<RectanglePlacement>> construct_neighbors_with_metadata(
        RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) override;

    void add_exploration_moves(const std::vector<RectanglePlacement> &solution, int L, double overlap_tolerance,
                               std::vector<std::vector<RectanglePlacement>> &neighbors,
                               std::vector<NeighborMetadata> &metadata, int max_neighbors);

    void generate_aggressive_moves(const std::vector<RectanglePlacement> &solution, int L,
                                   std::vector<std::vector<RectanglePlacement>> &neighbors,
                                   std::vector<NeighborMetadata> &metadata, int max_neighbors, int T);

    void generate_swap_focused_moves(RectangleFittingProblem &problem, const std::vector<RectanglePlacement> &solution,
                                     int L, std::vector<std::vector<RectanglePlacement>> &neighbors,
                                     std::vector<NeighborMetadata> &metadata, int max_neighbors, int T);

    void generate_conservative_moves(RectangleFittingProblem &problem, const std::vector<RectanglePlacement> &solution,
                                     int L, std::vector<std::vector<RectanglePlacement>> &neighbors,
                                     std::vector<NeighborMetadata> &metadata, int max_neighbors, int T);

    void generate_swap_moves(const std::vector<RectanglePlacement> &solution, int L,
                             std::vector<std::vector<RectanglePlacement>> &neighbors,
                             std::vector<NeighborMetadata> &metadata, int max_swaps);

private:
    std::vector<RectanglePlacement> generate_swap_move(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    std::vector<RectanglePlacement> generate_random_spread(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    long long calculate_total_overlap(
        const std::vector<RectanglePlacement>& solution);
    static bool is_neighbor_acceptable(const std::vector<RectanglePlacement>& neighbor,
                                   int L, double overlap_tolerance);

    bool is_acceptable(const std::vector<RectanglePlacement> &neighbor, int L, double overlap_tolerance);

    static bool has_overlaps(const std::vector<RectanglePlacement>& solution);
};

void reset_relaxed_temperature();

#endif