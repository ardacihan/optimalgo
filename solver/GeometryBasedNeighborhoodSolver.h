#ifndef GEOMETRYBASEDNEIGHBORHOODSOLVER_H
#define GEOMETRYBASEDNEIGHBORHOODSOLVER_H

#include "../problem/RectangleFittingProblem.h"
#include <vector>
#include "Solver.h"

class GeometryBasedNeighborhoodSolver : public Solver {
protected:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) override;
};

#endif // GEOMETRYBASEDNEIGHBORHOODSOLVER_H