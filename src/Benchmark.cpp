#include "Benchmark.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <unordered_map>
#include <map>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <unordered_set>

#include "problem/RectangleFittingProblem.h"
#include "problem/primitives/InstanceGenerator.h"
#include "solver/greedy/GreedySolver.h"
#include "solver/local_search/specialized_solvers/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.h"

struct BenchmarkConfig {
    int rectCount;
    int boxSize;
    int minW, maxW;
    int minH, maxH;
    int num_reruns;
    int max_rectangles_in_subproblem;
    std::string name;
};

struct SolutionMetadata {
    int box_size;
    int rect_count;
    int boxes_used;
    double utilization;
    std::string solver_name;
    double solve_time;
    int objective_value;
    std::string timestamp;
    int min_width;
    int max_width;
    int min_height;
    int max_height;
};

// ===== HELPER FUNCTIONS =====

static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

static double calculateUtilization(
    const std::vector<RectanglePlacement>& placements,
    int box_size)
{
    if (placements.empty()) return 0.0;

    std::unordered_set<int> boxes;
    double total_rect_area = 0.0;

    for (const auto& p : placements) {
        boxes.insert(p.box_id);
        total_rect_area += p.width * p.height;
    }

    double total_box_area = boxes.size() * box_size * box_size;
    return (total_rect_area / total_box_area) * 100.0;
}

static std::string occupancy_histogram(
    const std::vector<RectanglePlacement>& solution,
    int box_size)
{
    const double box_area = static_cast<double>(box_size * box_size);

    std::unordered_map<int, double> area_per_box;
    for (const auto& r : solution) {
        double area = static_cast<double>(r.get_actual_height() * r.get_actual_width());
        area_per_box[r.box_id] += area;
    }

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

// ===== SAVE SOLUTION FUNCTIONS =====

static bool saveSolutionXML(
    const std::string& filepath,
    const std::vector<RectanglePlacement>& placements,
    const SolutionMetadata& metadata)
{
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<PackingSolution>\n";

    file << "  <Metadata>\n";
    file << "    <BoxSize>" << metadata.box_size << "</BoxSize>\n";
    file << "    <RectangleCount>" << metadata.rect_count << "</RectangleCount>\n";
    file << "    <BoxesUsed>" << metadata.boxes_used << "</BoxesUsed>\n";
    file << "    <Utilization>" << std::fixed << std::setprecision(2)
         << metadata.utilization << "</Utilization>\n";
    file << "    <SolverName>" << metadata.solver_name << "</SolverName>\n";
    file << "    <SolveTime>" << std::fixed << std::setprecision(3)
         << metadata.solve_time << "</SolveTime>\n";
    file << "    <ObjectiveValue>" << metadata.objective_value << "</ObjectiveValue>\n";
    file << "    <Timestamp>" << metadata.timestamp << "</Timestamp>\n";
    file << "    <MinWidth>" << metadata.min_width << "</MinWidth>\n";
    file << "    <MaxWidth>" << metadata.max_width << "</MaxWidth>\n";
    file << "    <MinHeight>" << metadata.min_height << "</MinHeight>\n";
    file << "    <MaxHeight>" << metadata.max_height << "</MaxHeight>\n";
    file << "  </Metadata>\n";

    file << "  <Placements>\n";
    for (const auto& p : placements) {
        file << "    <Rectangle>\n";
        file << "      <Width>" << p.width << "</Width>\n";
        file << "      <Height>" << p.height << "</Height>\n";
        file << "      <X>" << p.x << "</X>\n";
        file << "      <Y>" << p.y << "</Y>\n";
        file << "      <BoxID>" << p.box_id << "</BoxID>\n";
        file << "      <Rotated>" << (p.rotated ? "true" : "false") << "</Rotated>\n";
        file << "    </Rectangle>\n";
    }
    file << "  </Placements>\n";

    file << "</PackingSolution>\n";
    file.close();

    return true;
}

static bool saveSolutionText(
    const std::string& filepath,
    const std::vector<RectanglePlacement>& placements,
    const SolutionMetadata& metadata)
{
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    file << "RECTANGLE PACKING SOLUTION\n";
    file << "==========================\n\n";

    file << "METADATA:\n";
    file << "---------\n";
    file << "Timestamp:        " << metadata.timestamp << "\n";
    file << "Solver:           " << metadata.solver_name << "\n";
    file << "Solve Time:       " << std::fixed << std::setprecision(3)
         << metadata.solve_time << " seconds\n";
    file << "Box Size:         " << metadata.box_size << "x" << metadata.box_size << "\n";
    file << "Rectangle Count:  " << metadata.rect_count << "\n";
    file << "Boxes Used:       " << metadata.boxes_used << "\n";
    file << "Utilization:      " << std::fixed << std::setprecision(2)
         << metadata.utilization << "%\n";
    file << "Objective Value:  " << metadata.objective_value << "\n";
    file << "Width Range:      [" << metadata.min_width << ", " << metadata.max_width << "]\n";
    file << "Height Range:     [" << metadata.min_height << ", " << metadata.max_height << "]\n";
    file << "\n";

    std::unordered_map<int, std::vector<RectanglePlacement>> boxes_map;
    for (const auto& p : placements) {
        boxes_map[p.box_id].push_back(p);
    }

    file << "PLACEMENTS:\n";
    file << "-----------\n";

    for (const auto& [box_id, box_placements] : boxes_map) {
        file << "\nBox " << (box_id + 1) << " (" << box_placements.size() << " rectangles):\n";

        double box_area = metadata.box_size * metadata.box_size;
        double used_area = 0.0;
        for (const auto& p : box_placements) {
            used_area += p.width * p.height;
        }
        double box_util = (used_area / box_area) * 100.0;
        file << "  Utilization: " << std::fixed << std::setprecision(2) << box_util << "%\n";

        for (size_t i = 0; i < box_placements.size(); i++) {
            const auto& p = box_placements[i];
            file << "  Rectangle " << std::setw(3) << (i + 1) << ": "
                 << std::setw(2) << p.width << "x" << std::setw(2) << p.height
                 << " at (" << std::setw(3) << p.x << ", " << std::setw(3) << p.y << ")"
                 << (p.rotated ? " [Rotated]" : "")
                 << "\n";
        }
    }

    file.close();
    return true;
}

// ===== BENCHMARK FUNCTIONS =====

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

        out << "Occupancy: "
            << occupancy_histogram(result, BOX_SIZE)
            << "\n\n";

        out.flush();

        // Save solution to XML
        SolutionMetadata metadata;
        metadata.box_size = BOX_SIZE;
        metadata.rect_count = RECT_COUNT;
        metadata.boxes_used = boxes;
        metadata.utilization = calculateUtilization(result, BOX_SIZE);
        metadata.solver_name = name;
        metadata.solve_time = time;
        metadata.objective_value = objective;
        metadata.timestamp = getCurrentTimestamp();
        metadata.min_width = MIN_W;
        metadata.max_width = MAX_W;
        metadata.min_height = MIN_H;
        metadata.max_height = MAX_H;

        std::string xml_filename = "solution_light_" + name + ".xml";
        if (saveSolutionXML(xml_filename, result, metadata)) {
            std::cout << "Saved solution: " << xml_filename << std::endl;
        }
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
    std::vector<BenchmarkConfig> configs = {
        {10000, 100, 5, 20, 5, 20, 0, 200, "Big"},
        {10000, 100, 20, 50, 5, 10, 0, 100, "SmallerSubproblem"},
        {5000, 200, 20, 50, 5, 10, 0, 100, "BiggerBox"},
        {5000, 200, 20, 50, 20, 100, 0, 100, "BiggerRect"}
    };

    const int T = 1000;

    std::ofstream out("benchmark_results_big.txt", std::ios::out);
    if (!out) {
        std::cerr << "Failed to open benchmark_results_big.txt" << std::endl;
        return;
    }

    out << "RECTANGLE PACKING – HEAVY BENCHMARK\n";
    out << "====================================\n\n";

    for (const auto& config : configs) {
        const int RECT_COUNT = config.rectCount;
        const int BOX_SIZE = config.boxSize;
        const int MIN_W = config.minW;
        const int MAX_W = config.maxW;
        const int MIN_H = config.minH;
        const int MAX_H = config.maxH;
        const int NUM_RERUNS = config.num_reruns;
        const int MAX_SUBPROBLEM = config.max_rectangles_in_subproblem;

        out << "CONFIG: " << config.name << "\n";
        out << "Rectangles=" << RECT_COUNT
            << " BoxSize=" << BOX_SIZE
            << " W=[" << MIN_W << "," << MAX_W << "]"
            << " H=[" << MIN_H << "," << MAX_H << "]\n";
        out << "Parameters: NUM_RERUNS=" << NUM_RERUNS
            << " MAX_SUBPROBLEM=" << MAX_SUBPROBLEM
            << " T=" << T << "\n\n";

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

            // Save solution to XML
            SolutionMetadata metadata;
            metadata.box_size = BOX_SIZE;
            metadata.rect_count = RECT_COUNT;
            metadata.boxes_used = boxes;
            metadata.utilization = calculateUtilization(result, BOX_SIZE);
            metadata.solver_name = name;
            metadata.solve_time = time;
            metadata.objective_value = objective;
            metadata.timestamp = getCurrentTimestamp();
            metadata.min_width = MIN_W;
            metadata.max_width = MAX_W;
            metadata.min_height = MIN_H;
            metadata.max_height = MAX_H;

            std::string xml_filename = "solution_heavy_" + config.name + "_" + name + ".xml";
            if (saveSolutionXML(xml_filename, result, metadata)) {
                std::cout << "Saved solution: " << xml_filename << std::endl;
            }
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

        out << "========================================\n\n";
        std::cout << "Completed configuration: " << config.name << std::endl;
    }

    out << "\nAll benchmarks completed.\n";
    out.close();
    std::cout << "All benchmarks completed. Results written to benchmark_results_big.txt" << std::endl;
}