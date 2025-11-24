//
// Created by arda on 13.11.25.
//

#ifndef OPTIMALGO_RELAXEDGEOMETRYBASEDNEIGHBORHOODSOLVER_H
#define OPTIMALGO_RELAXEDGEOMETRYBASEDNEIGHBORHOODSOLVER_H

#include "../problem/RectangleFittingProblem.h"
#include <vector>

#include "Solver.h"

class RelaxedGeometryBasedNeighborhoodSolver : public Solver {
protected:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);
};

#endif //OPTIMALGO_RELAXEDGEOMETRYBASEDNEIGHBORHOODSOLVER_H