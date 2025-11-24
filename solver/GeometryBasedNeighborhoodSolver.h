#ifndef GEOMATRYBASEDNEIGHBORHOODSOLVER_H
#define GEOMATRYBASEDNEIGHBORHOODSOLVER_H

#include "../problem/RectangleFittingProblem.h"
#include <vector>

#include "Solver.h"

class GeometryBasedNeighborhoodSolver : public Solver {
protected:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) override;
};

#endif //  GEOMATRYBASEDNEIGHBORHOODSOLVER_H
