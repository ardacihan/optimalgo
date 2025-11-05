#ifndef RULEBASEDNEIGHBORHOODSOLVER_H
#define RULEBASEDNEIGHBORHOODSOLVER_H

#include "RectangleFittingProblem.h"
#include <vector>

class RuleBasedNeighborhoodSolver {
public:
    // Main solve methods (same interface as GeometryBased!)
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int max_steps);
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);

private:
    // Apply greedy placement rule to rectangles in given order
    std::vector<RectanglePlacement> apply_greedy_placement(
        const std::vector<RectanglePlacement>& rectangles_in_order,
        int L);
};

#endif // RULEBASEDNEIGHBORHOODSOLVER_H