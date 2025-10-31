//
// Solver.h
//

#ifndef SOLVER_H
#define SOLVER_H

#include "RectangleFittingProblem.h"
#include <vector>

class GeometryBasedNeighborhoodSolver {
public:
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int max_steps);
    
private:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);
};

#endif // SOLVER_H
