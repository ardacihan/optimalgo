//
// Created by arda on 11/24/25.
//


#ifndef OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H
#define OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H


#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "../problem/RectangleFittingProblem.h"


class RectangleFittingProblemSolver {
public:
    virtual ~RectangleFittingProblemSolver() {}

    virtual std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                                  int max_rectangle_in_subproblem, int T) {return std::vector<RectanglePlacement>();};

    virtual std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem, int T) {return std::vector<RectanglePlacement>();};

};



#endif //OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H