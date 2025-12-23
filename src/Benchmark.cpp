#include "Benchmark.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include "problem/RectangleFittingProblem.h"
#include "problem/primitives/InstanceGenerator.h"
#include "solver/greedy/GreedySolver.h"
#include "solver/local_search/specialized_solvers/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.h"

static double elapsed_seconds(
    const std::chrono::steady_clock::time_point& start,
    const std::chrono::steady_clock::time_point& end)
{
    return std::chrono::duration<double>(end - start).count();
}

static int count_boxes(const std::vector<RectanglePlacement>& solution) {
    std::unordered_set<int> boxes;
    for (const auto& r : solution) {
        boxes.insert(r.box_id);
    }
    return (int)boxes.size();
}

void Benchmark::runNormal() {
    const int RECT_COUNT = 1000;
    const int BOX_SIZE   = 80;
    const int MIN_W = 5;
    const int MAX_W = 20;
    const int MIN_H = 5;
    const int MAX_H = 20;

    const int NUM_RERUNS = 0;
    const int MAX_SUBPROBLEM = 50;
    const int T = 1000;

    std::ofstream out("benchmark_results.txt", std::ios::out);
    if (!out) {
        std::cerr << "Failed to open benchmark_results.txt" << std::endl;
        return;
    }

    out << "RECTANGLE PACKING – LIGHT BENCHMARK\n";
    out << "===================================\n";
    out << "Rectangles=" << RECT_COUNT
        << " BoxSize=" << BOX_SIZE
        << " W=[" << MIN_W << "," << MAX_W << "]"
        << " H=[" << MIN_H << "," << MAX_H << "]\n\n";

    InstanceGenerator generator(
        BOX_SIZE,
        MIN_W, MAX_W,
        MIN_H, MAX_H
    );

    std::vector<RectanglePlacement> initial =
        generator.generate_rectangles(RECT_COUNT);

    auto run_solver = [&](const std::string& name, auto&& solver_fn) {
        RectangleFittingProblem problem(BOX_SIZE, initial);

        auto start = std::chrono::steady_clock::now();
        std::vector<RectanglePlacement> result = solver_fn(problem);
        auto end = std::chrono::steady_clock::now();

        double time = elapsed_seconds(start, end);
        int objective = problem.objective(result);
        int boxes = count_boxes(result);

        out << name
            << " | time=" << time << "s"
            << " | objective=" << objective
            << " | boxes=" << boxes
            << "\n";

        out.flush();
    };

    {
        GeometryBasedNeighborhoodSolver solver;
        run_solver("GeometryBased",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        RuleBasedNeighborhoodSolver solver;
        run_solver("RuleBased",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        RelaxedGeometryBasedNeighborhoodSolver solver;
        run_solver("RelaxedGeometry",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(0);
        run_solver("GreedyBiggestFirst",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(1);
        run_solver("GreedySmallestFirst",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(2);
        run_solver("GreedyBestFit",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    out << "\nBenchmark complete.\n";
    out.close();
}

void Benchmark::runHeavy() {
    const int RECT_COUNT = 5000;
    const int BOX_SIZE   = 80;
    const int MIN_W = 5;
    const int MAX_W = 20;
    const int MIN_H = 5;
    const int MAX_H = 20;

    const int NUM_RERUNS = 0;
    const int MAX_SUBPROBLEM = 50;
    const int T = 5000;

    std::ofstream out("benchmark_results.txt", std::ios::out);
    if (!out) {
        std::cerr << "Failed to open benchmark_results.txt" << std::endl;
        return;
    }

    out << "RECTANGLE PACKING – LIGHT BENCHMARK\n";
    out << "===================================\n";
    out << "Rectangles=" << RECT_COUNT
        << " BoxSize=" << BOX_SIZE
        << " W=[" << MIN_W << "," << MAX_W << "]"
        << " H=[" << MIN_H << "," << MAX_H << "]\n\n";

    InstanceGenerator generator(
        BOX_SIZE,
        MIN_W, MAX_W,
        MIN_H, MAX_H
    );

    std::vector<RectanglePlacement> initial =
        generator.generate_rectangles(RECT_COUNT);

    auto run_solver = [&](const std::string& name, auto&& solver_fn) {
        RectangleFittingProblem problem(BOX_SIZE, initial);

        auto start = std::chrono::steady_clock::now();
        std::vector<RectanglePlacement> result = solver_fn(problem);
        auto end = std::chrono::steady_clock::now();

        double time = elapsed_seconds(start, end);
        int objective = problem.objective(result);
        int boxes = count_boxes(result);

        out << name
            << " | time=" << time << "s"
            << " | objective=" << objective
            << " | boxes=" << boxes
            << "\n";

        out.flush();
    };

    {
        GeometryBasedNeighborhoodSolver solver;
        run_solver("GeometryBased",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        RuleBasedNeighborhoodSolver solver;
        run_solver("RuleBased",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        RelaxedGeometryBasedNeighborhoodSolver solver;
        run_solver("RelaxedGeometry",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(0);
        run_solver("GreedyBiggestFirst",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(1);
        run_solver("GreedySmallestFirst",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    {
        GreedySolver solver;
        solver.set_selection_strategy(2);
        run_solver("GreedyBestFit",
            [&](RectangleFittingProblem& p) {
                return solver.solve(p, NUM_RERUNS, MAX_SUBPROBLEM, T);
            });
    }

    out << "\nBenchmark complete.\n";
    out.close();
}
