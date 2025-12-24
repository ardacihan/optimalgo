#include "Benchmark.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <map>
#include <sstream>


#include "problem/RectangleFittingProblem.h"
#include "problem/primitives/InstanceGenerator.h"
#include "solver/greedy/GreedySolver.h"
#include "solver/local_search/specialized_solvers/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.h"

static std::string occupancy_histogram(
    const std::vector<RectanglePlacement>& solution,
    int box_size)
{
    const double box_area = static_cast<double>(box_size * box_size);

    // Accumulate used area per box
    std::unordered_map<int, double> area_per_box;
    for (const auto& r : solution) {
        double area = static_cast<double>(r.get_actual_height() * r.get_actual_width());
        area_per_box[r.box_id] += area;
    }

    // Bins: label -> count
    std::map<std::string, int> bins = {
        {"100-95", 0},
        {"94-90", 0},
        {"89-85", 0},
        {"85-80", 0},
        {"<80",    0}
    };

    for (const auto& [box_id, used_area] : area_per_box) {
        double pct = (used_area / box_area) * 100.0;

        if (pct >= 95.0)       bins["100-95"]++;
        else if (pct >= 90.0)  bins["94-90"]++;
        else if (pct >= 85.0)  bins["89-85"]++;
        else if (pct >= 80.0)  bins["85-80"]++;
        else                   bins["<80"]++;
    }

    std::ostringstream oss;
    bool first = true;
    for (const auto& [label, count] : bins) {
        if (!first) oss << " ; ";
        first = false;
        oss << label << " : " << count;
    }

    return oss.str();
}


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

        // NEW: occupancy distribution
        out << "Occupancy: "
            << occupancy_histogram(result, BOX_SIZE)
            << "\n\n";

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
    const int RECT_COUNT = 20000;
    const int BOX_SIZE   = 80;
    const int MIN_W = 5;
    const int MAX_W = 20;
    const int MIN_H = 5;
    const int MAX_H = 20;

    const int NUM_RERUNS = 0;
    const int MAX_SUBPROBLEM = 50;
    const int T = 5000;

    std::ofstream out("benchmark_results_heavy.txt", std::ios::out);
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

        out << "Occupancy: "
            << occupancy_histogram(result, BOX_SIZE)
            << "\n\n";

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

void Benchmark::runBig() {
    const int RECT_COUNT = 10000;
    const int BOX_SIZE   = 200;
    const int MIN_W = 5;
    const int MAX_W = 20;
    const int MIN_H = 5;
    const int MAX_H = 20;

    const int NUM_RERUNS = 0;
    const int MAX_SUBPROBLEM = 200;
    const int T = 5000;

    std::ofstream out("benchmark_results_big.txt", std::ios::out);
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

        // NEW: occupancy distribution
        out << "Occupancy: "
            << occupancy_histogram(result, BOX_SIZE)
            << "\n\n";

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


