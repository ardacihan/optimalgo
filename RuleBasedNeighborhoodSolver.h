#ifndef RULEBASEDNEIGHBORHOODSOLVER_H
#define RULEBASEDNEIGHBORHOODSOLVER_H

#include "RectangleFittingProblem.h"
#include <vector>
#include "Solver.h"

class RuleBasedNeighborhoodSolver : Solver {
public:
    // Main solve methods (same interface as GeometryBased!)

    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int max_steps, int rectangles_in_subproblem);

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                                      int max_rectangle_in_subproblem);

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem);

    std::vector<RectanglePlacement> solve_one_step_recursive(RectangleFittingProblem &problem);

    std::vector<RectanglePlacement> apply_greedy_placement_indexed(const std::vector<int> &rect_indices,
                                                                   const std::vector<std::pair<int, int>> &rect_dims,
                                                                   int L);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);

private:
    // Apply greedy placement rule to rectangles in given order
    std::vector<RectanglePlacement> apply_greedy_placement(
        const std::vector<RectanglePlacement>& rectangles_in_order,
        int L);
};

#endif // RULEBASEDNEIGHBORHOODSOLVER_H