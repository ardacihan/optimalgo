//
// Created by arda on 12/5/25.
//

#ifndef OPTIMALGO_BENCHMARK_H
#define OPTIMALGO_BENCHMARK_H
#include "gui/InstanceGenerator.h"
#include "problem/RectangleFittingProblem.h"
#include "solver/local_search/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.h"

#include "solver/greedy/GreedySolver.h"



class Benchmark {

    GeometryBasedNeighborhoodSolver solver;
    RuleBasedNeighborhoodSolver solver2;
    RelaxedGeometryBasedNeighborhoodSolver solver3;

    GreedySolver solver4;

    InstanceGenerator generator;


};


#endif //OPTIMALGO_BENCHMARK_H