#ifndef RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H
#define RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>

class RelaxedGeometryBasedNeighborhoodSolver : public GeometryBasedNeighborhoodSolver {
public:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) override;


private:
    std::vector<std::vector<RectanglePlacement>> construct_overlapping_neighbors(
        RectangleFittingProblem &problem, int T);

    // Temperature-based parameter calculations
    double calculate_max_overlap_ratio(int T);
    int calculate_max_overlap_area(int T, int L);
    int calculate_overlap_offset(int T, int rect_size);
    int calculate_perturbation_range(int T);

};

void reset_relaxed_temperature();


#endif // RELAXED_GEOMETRY_BASED_NEIGHBORHOOD_SOLVER_H