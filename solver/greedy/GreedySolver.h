#ifndef OPTIMALGO_GREEDYSOLVER_H
#define OPTIMALGO_GREEDYSOLVER_H

#include "problem/RectangleFittingProblem.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <memory>

#include "solver/RectangleFittingProblemSolver.h"

class SelectionStrategy {
public:
    virtual ~SelectionStrategy() {}
    virtual RectanglePlacement select_rectangle(RectangleFittingProblem &problem) = 0;
};

class BiggestFirstSelectionStrategy : public SelectionStrategy {
public:
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem) override;
};

class MaximizeContactSelectionStrategy : public SelectionStrategy {
public:
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem) override;

private:
    double calculate_simple_score(const RectanglePlacement& rect, bool is_alone);
};


class GreedySolver : public RectangleFittingProblemSolver {
public:
    GreedySolver() {
        selection_strategy = std::make_unique<BiggestFirstSelectionStrategy>();
    }

    std::unique_ptr<SelectionStrategy> selection_strategy;

    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                          int max_rectangle_in_subproblem) override;

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem) override;

    void set_selection_strategy(int strategy_type) {
        if (strategy_type == 0) {
            selection_strategy = std::make_unique<BiggestFirstSelectionStrategy>();
        } else {
            selection_strategy = std::make_unique<MaximizeContactSelectionStrategy>();
        }
    }

protected:
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem) {
        return selection_strategy->select_rectangle(problem);
    }

    void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle);

private:
    RectanglePlacement place_rectangle(RectangleFittingProblem &problem,
                                      const RectanglePlacement &selected_rect,
                                      std::vector<std::vector<int>>& occupancy_grids,
                                      int& next_box_id);

    bool collides_with_occupancy(int x, int y, int w, int h,
                                const std::vector<int>& grid, int L);

    void mark_occupied(const RectanglePlacement& placement,
                      std::vector<std::vector<int>>& occupancy_grids,
                      int rect_idx, int L);
};

#endif //OPTIMALGO_GREEDYSOLVER_H