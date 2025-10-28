#ifndef OPTIMIZATIONPROBLEM_H
#define OPTIMIZATIONPROBLEM_H

#include <vector>
#include <any>
#include <set>
#include <functional>
#include "Input.h"


class OptimizationProblem {
public:
    std::vector<Input> inputs;
    std::set<std::function<bool(Input)>> constraints;

    virtual int objective(const Input& input) = 0;
    virtual ~OptimizationProblem() = default;


    bool apply_constraints(const Input& input) {
        for (auto&& constraint : constraints) {
            if (!constraint(input)) {
                return false;
            }
        }
        return true;
    }

    void add_constraint(const std::function<bool(Input)>& constraint) {
        //todo fix this constraints.insert(constraint);
    }
};

#endif // OPTIMIZATIONPROBLEM_H