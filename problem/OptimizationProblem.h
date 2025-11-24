//
// OptimizationProblem.h
//

#ifndef OPTIMIZATIONPROBLEM_H
#define OPTIMIZATIONPROBLEM_H

template <typename Solution>
class OptimizationProblem {
public:
    virtual ~OptimizationProblem() = default;

    // Returns the objective value for a given solution (higher is better)
    virtual int objective(const Solution& solution) = 0;

    // Checks if a solution is legal/feasible
    virtual bool solution_legal(const Solution& solution) const = 0;

    // Get the current solution
    virtual Solution get_current_solution() const = 0;

    // Set the current solution
    virtual void set_current_solution(const Solution& solution) = 0;
};

#endif // OPTIMIZATIONPROBLEM_H
