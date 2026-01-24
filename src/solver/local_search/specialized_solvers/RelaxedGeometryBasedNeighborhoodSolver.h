#ifndef RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H
#define RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>

class RelaxedGeometryBasedNeighborhoodSolver : public GeometryBasedNeighborhoodSolver {
public:
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;

    void add_exploration_moves(const std::vector<RectanglePlacement> &solution, int L, long long box_capacity,
                               std::vector<std::vector<RectanglePlacement>> &neighbors, int max_neighbors);

    void add_fixing_moves(const std::vector<RectanglePlacement> &solution, int L, long long box_capacity,
                          std::vector<std::vector<RectanglePlacement>> &neighbors, int max_neighbors);

    void add_exploration_moves(const std::vector<RectanglePlacement> &solution, int L, double overlap_tolerance,
                               std::vector<std::vector<RectanglePlacement>> &neighbors, int max_neighbors);

private:
    bool is_acceptable(const std::vector<RectanglePlacement> &neighbor, int L, double overlap_tolerance);

    bool has_overlaps(const std::vector<RectanglePlacement>& solution);
};

#endif