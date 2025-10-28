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

    virtual int objective(const Input& input) = 0;
    virtual ~OptimizationProblem() = default;


    virtual bool apply_constraints(const Input& input) { return false;   }
};

#endif // OPTIMIZATIONPROBLEM_H