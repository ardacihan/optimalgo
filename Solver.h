#ifndef SOLVER_H
#define SOLVER_H

#include "OptimizationProblem.h"
#include <memory>

class Solver {
public:
    virtual ~Solver() = default;
    virtual Input solve(OptimizationProblem& problem, const Input& initial_solution) = 0;

};

class NeighborhoodSolver : Solver {public:
    Input solve(Input& input);
    std::vector<Input> construct_neighbors(Input& input);
};

class GeometryBasedNeighborhoodSolver : Solver {
public:
    Input solve(Input& input);
    std::vector<Input> construct_neighbors(Input& input);
};

class RuleBasedNeighborhoodSolver : Solver {
public:
    Input solve(Input& input);
    std::vector<Input> construct_neighbors(Input& input);
};


class SemiOverlappingAllowedNeighborhoodSolver : Solver {
public:
    Input solve(Input& input);
    std::vector<Input> construct_neighbors(Input& input);
};


class GreedySolver: Solver {
public:
    Input solve(Input& input);
    std::vector<Input> construct_neighbors(Input& input);
};

#endif // SOLVER_H