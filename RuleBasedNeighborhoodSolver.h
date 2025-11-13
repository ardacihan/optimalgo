#ifndef RULEBASEDNEIGHBORHOODSOLVER_H
#define RULEBASEDNEIGHBORHOODSOLVER_H

#include "RectangleFittingProblem.h"
#include <vector>
#include "Solver.h"

class RuleBasedNeighborhoodSolver : public Solver {
protected:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) override;

private:
    std::vector<RectanglePlacement> apply_greedy_placement_indexed(
        const std::vector<int>& rect_indices,
        const std::vector<std::pair<int, int>>& rect_dims,
        int L);
};
#endif // RULEBASEDNEIGHBORHOODSOLVER_H