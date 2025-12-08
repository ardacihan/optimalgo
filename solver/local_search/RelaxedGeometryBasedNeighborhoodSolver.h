
// ============================================================================
// FILE: solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.h (UPDATED)
// ============================================================================
#ifndef RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H
#define RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>

class RelaxedGeometryBasedNeighborhoodSolver : public GeometryBasedNeighborhoodSolver {
public:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;
    
    std::vector<std::vector<RectanglePlacement>> construct_neighbors_with_metadata(
        RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) override;

private:
    std::vector<RectanglePlacement> generate_swap_move(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    std::vector<RectanglePlacement> generate_random_spread(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    long long calculate_total_overlap(
        const std::vector<RectanglePlacement>& solution);
};

void reset_relaxed_temperature();

#endif