#ifndef OPTIMIZATIONPROBLEM_H
#define OPTIMIZATIONPROBLEM_H

#include <vector>
#include <any>
#include <set>
#include <functional>
#include "Input.h"


class OptimizationProblem {
public:
    Input current_solution;

     int objective(const Input& input);
     ~OptimizationProblem() = default;


    bool apply_constraints(const Input& input) { return false;   }
};

#endif // OPTIMIZATIONPROBLEM_H