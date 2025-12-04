#ifndef RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H
#define RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>

class RelaxedGeometryBasedNeighborhoodSolver : public GeometryBasedNeighborhoodSolver {
public:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;

private:
    std::vector<std::vector<RectanglePlacement>> construct_overlapping_neighbors(
        RectangleFittingProblem &problem, int T);

    // Helper function for quadrant perturbation
    std::vector<RectanglePlacement> generate_quadrant_perturbation(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    // Helper function for swap moves
    std::vector<RectanglePlacement> generate_swap_move(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    // Helper function for random spread (for high temperatures)
    std::vector<RectanglePlacement> generate_random_spread(
        const std::vector<RectanglePlacement>& solution,
        int L, int T);

    // Helper to calculate total overlap in a solution
    long long calculate_total_overlap(
        const std::vector<RectanglePlacement>& solution);
};

void reset_relaxed_temperature();

#endif // RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H