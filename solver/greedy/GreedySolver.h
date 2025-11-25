//
// Created by arda on 11/24/25.
//

#ifndef OPTIMALGO_GREEDYSOLVER_H
#define OPTIMALGO_GREEDYSOLVER_H
#include "problem/RectangleFittingProblem.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

class SelectionStrategy {
public:
    virtual ~SelectionStrategy() {}
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem){}


};

class BiggestFirstSelectionStrategy : public SelectionStrategy {
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem);
};



class MaximizeContactSelectionStrategy : public SelectionStrategy {
public:
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem) {}

private:
    double calculate_simple_score(const RectanglePlacement& rect, bool is_alone) {}
};

class PlacementStrategy {
public:
    virtual ~PlacementStrategy() {}
    void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle){}
};


class GreedySolver {
public:
    virtual ~GreedySolver() {}

    SelectionStrategy selection_strategy;
    PlacementStrategy placement_strategy;

    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                          int max_rectangle_in_subproblem);

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem);
protected:
    RectanglePlacement select_rectangle(RectangleFittingProblem &problem) { return selection_strategy.select_rectangle(problem);}

    void place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle)
    { placement_strategy.place_rectangle(problem, rectangle); }



};









#endif //OPTIMALGO_GREEDYSOLVER_H