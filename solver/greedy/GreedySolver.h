#ifndef OPTIMALGO_GREEDYSOLVER_H
#define OPTIMALGO_GREEDYSOLVER_H
#include "problem/RectangleFittingProblem.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <memory>

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

class PlacementStrategy {
public:
    virtual ~PlacementStrategy() {}
    virtual void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle) = 0;
};

class RightTopPlacementStrategy : public PlacementStrategy {
public:
    RightTopPlacementStrategy() {}
    void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle) override;
};

class GreedySolver {
public:
    GreedySolver() {
        selection_strategy = std::make_unique<BiggestFirstSelectionStrategy>();
        placement_strategy = std::make_unique<RightTopPlacementStrategy>();
    }

    std::unique_ptr<SelectionStrategy> selection_strategy;
    std::unique_ptr<PlacementStrategy> placement_strategy;

    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                          int max_rectangle_in_subproblem);

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem);

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

    void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle) {
        placement_strategy->place_rectangle(problem, rectangle);
    }
};

#endif //OPTIMALGO_GREEDYSOLVER_H