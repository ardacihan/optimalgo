
// ============================================================================
// FILE: solver/local_search/RuleBasedNeighborhoodSolver.h (UPDATED)
// ============================================================================
#ifndef RULEBASEDNEIGHBORHOODSOLVER_H
#define RULEBASEDNEIGHBORHOODSOLVER_H

#include "../../problem/RectangleFittingProblem.h"
#include <vector>
#include "LocalSearchSolver.h"

class RuleBasedNeighborhoodSolver : public LocalSearchSolver {
protected:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;
    
    std::vector<std::vector<RectanglePlacement>> construct_neighbors_with_metadata(
        RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) override;

private:
    std::vector<RectanglePlacement> apply_greedy_placement_indexed(
        const std::vector<int>& rect_indices,
        const std::vector<std::pair<int, int>>& rect_dims,
        int L);
};

#endif