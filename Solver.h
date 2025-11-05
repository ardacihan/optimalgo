//
// Solver.h
//

#ifndef SOLVER_H
#define SOLVER_H

#include "RectangleFittingProblem.h"
#include <vector>

class GeometryBasedNeighborhoodSolver {
public:
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem);

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int max_steps, int reruns_left);

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem);

    std::vector<RectanglePlacement> solve_one_step_recursive(RectangleFittingProblem &problem);

    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>> splitRectanglesByBoxId(
        const std::vector<RectanglePlacement> &placements);

private:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);
};

#endif // SOLVER_H
