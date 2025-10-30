#ifndef SOLVER_H
#define SOLVER_H

#include "OptimizationProblem.h"
#include "Input.h"
#include <memory>
#include <vector>
#include <set>

#include "RectangleFittingProblem.h"
#include "RectanglePlacement.h"

class Solver {
public:
    ~Solver() = default;
    std::unique_ptr<Input> solve(OptimizationProblem& problem, int max_steps);
};

class NeighborhoodSolver : public Solver {
public:
    std::unique_ptr<Input> solve(OptimizationProblem& problem,int max_steps);
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);
};

class GeometryBasedNeighborhoodSolver : public NeighborhoodSolver {
public:
    std::vector<RectanglePlacement> solve(RectangleFittingProblem& problem,int max_steps);

    std::vector<RectanglePlacement> solve_timeboxed(RectangleFittingProblem &problem, double time_limit_seconds);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem) override;

    std::vector<RectanglePlacement> select_next_solution(
        RectangleFittingProblem &problem, std::vector<std::vector<RectanglePlacement>> neighborhood);
};

class RuleBasedNeighborhoodSolver : public NeighborhoodSolver {
protected:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution);
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem) override;
};

class GreedySolver : public Solver {
public:
    std::unique_ptr<Input> solve(OptimizationProblem& problem, const Input& initial_solution);
};



#endif // SOLVER_H