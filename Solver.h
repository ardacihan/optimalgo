#ifndef SOLVER_H
#define SOLVER_H

#include "OptimizationProblem.h"
#include "Input.h"
#include <memory>
#include <vector>

class Solver {
public:
    virtual ~Solver() = default;
    virtual std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution) = 0;
};

class NeighborhoodSolver : public Solver {
public:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution) override;
    virtual std::vector<std::unique_ptr<Input>> construct_neighbors(OptimizationProblem& problem, const Input& input) = 0;
};

class GeometryBasedNeighborhoodSolver : public NeighborhoodSolver {
public:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution) override;
    std::vector<std::unique_ptr<Input>> construct_neighbors(OptimizationProblem& problem, const Input& input) override;
};

class RuleBasedNeighborhoodSolver : public NeighborhoodSolver {
protected:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution) override;
    std::vector<std::unique_ptr<Input>> construct_neighbors(OptimizationProblem& problem, const Input& input) override;
};

class GreedySolver : public Solver {
public:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution) override;
};

#endif // SOLVER_H