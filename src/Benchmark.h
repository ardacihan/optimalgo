#ifndef ENHANCED_TEST_ENVIRONMENT_H
#define ENHANCED_TEST_ENVIRONMENT_H

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <random>
#include <algorithm>
#include <numeric>
#include "solver/RectangleFittingProblemSolver.h"
#include "problem/RectangleFittingProblem.h"
#include "problem/primitives/InstanceGenerator.h"
// ============================================================================
// FILE: benchmark.cpp - Comprehensive Solver Benchmark
// ============================================================================
#include <iostream>
#include <cmath>
#include "solver/local_search/specialized_solvers/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/specialized_solvers/RelaxedGeometryBasedNeighborhoodSolver.h"
#include "solver/greedy/GreedySolver.h"

// Test configuration tuple: (num_instances, num_rectangles, min_w, max_w, min_h, max_h, box_length)
struct TestConfig {
    int num_instances;
    int num_rectangles;
    int min_width;
    int max_width;
    int min_height;
    int max_height;
    int box_length;

    std::string to_string() const {
        return "R" + std::to_string(num_rectangles) +
               "_W" + std::to_string(min_width) + "-" + std::to_string(max_width) +
               "_H" + std::to_string(min_height) + "-" + std::to_string(max_height) +
               "_L" + std::to_string(box_length);
    }
};

struct BenchmarkResult {
    std::string solver_name;
    std::string config_name;
    int instance_id;
    int objective_value;
    double cpu_time_seconds;
    int num_boxes_used;
    bool is_feasible;

    // Statistics from the solution
    int overlaps;
    int out_of_bounds;

    // For determining best solver
    bool is_best = false;
};

class BenchmarkRunner {
public:
    std::vector<TestConfig> configs;
    std::vector<BenchmarkResult> results;
    std::ofstream txt_file;

    // Measure CPU thread time
    double measure_cpu_time(std::function<void()> func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> diff = end - start;
        return diff.count();
    }

    // Analyze solution quality
    void analyze_solution(const std::vector<RectanglePlacement>& solution,
                         int L, BenchmarkResult& result) {
        result.overlaps = 0;
        result.out_of_bounds = 0;
        result.is_feasible = true;

        int n = solution.size();

        // Count boxes
        std::set<int> boxes;
        for (const auto& rect : solution) {
            boxes.insert(rect.box_id);
        }
        result.num_boxes_used = boxes.size();

        // Check bounds
        for (const auto& rect : solution) {
            int w = rect.get_actual_width();
            int h = rect.get_actual_height();
            if (rect.x < 0 || rect.y < 0 || rect.x + w > L || rect.y + h > L) {
                result.out_of_bounds++;
                result.is_feasible = false;
            }
        }

        // Check overlaps
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                if (solution[i].box_id == solution[j].box_id &&
                    solution[i].collides(solution[j])) {
                    result.overlaps++;
                    result.is_feasible = false;
                }
            }
        }
    }

    // Find best solver for a specific instance
    void find_best_solver_for_instance(const std::string& config_name, int instance_id) {
        // Collect all results for this specific instance
        std::vector<BenchmarkResult*> instance_results;
        for (auto& result : results) {
            if (result.config_name == config_name && result.instance_id == instance_id) {
                instance_results.push_back(&result);
            }
        }

        if (instance_results.empty()) return;

        // Find the best result (minimum boxes, tie-break with maximum objective)
        BenchmarkResult* best_result = nullptr;
        for (auto& result : instance_results) {
            if (!result->is_feasible) continue; // Skip infeasible solutions

            if (best_result == nullptr) {
                best_result = result;
            } else if (result->num_boxes_used < best_result->num_boxes_used) {
                best_result = result;
            } else if (result->num_boxes_used == best_result->num_boxes_used) {
                if (result->objective_value > best_result->objective_value) {
                    best_result = result;
                }
            }
        }

        // Mark the best result
        if (best_result != nullptr) {
            best_result->is_best = true;
        }
    }

    BenchmarkRunner(const std::string& filename) {
        // Open text file in the same directory as the executable
        txt_file.open(filename);
        if (!txt_file.is_open()) {
            std::cerr << "Error: Could not open output file: " << filename << std::endl;
            return;
        }

        // Write header
        txt_file << "========================================\n";
        txt_file << "RECTANGLE PACKING SOLVER BENCHMARK\n";
        txt_file << "========================================\n\n";
    }

    ~BenchmarkRunner() {
        if (txt_file.is_open()) {
            txt_file.close();
        }
    }

    void add_config(const TestConfig& config) {
        configs.push_back(config);
    }

    void run_benchmark() {
        std::cout << "\n=== Starting Benchmark ===\n" << std::endl;

        // Write date to file
        txt_file << "Date: " << get_current_time() << "\n\n";
        txt_file.flush();

        for (const auto& config : configs) {
            std::cout << "\n--- Configuration: " << config.to_string() << " ---" << std::endl;
            std::cout << "Generating " << config.num_instances << " instances..." << std::endl;

            txt_file << "\n=== Configuration: " << config.to_string() << " ===\n";
            txt_file << "Instances: " << config.num_instances << "\n";
            txt_file << "Rectangles: " << config.num_rectangles << "\n";
            txt_file << "Dimensions: " << config.min_width << "-" << config.max_width
                    << " x " << config.min_height << "-" << config.max_height << "\n";
            txt_file << "Box Length: " << config.box_length << "\n\n";
            txt_file.flush();

            // Generate instances
            InstanceGenerator generator(config.box_length, config.min_width, config.max_width,
                                      config.min_height, config.max_height);

            std::vector<std::vector<RectanglePlacement>> instances;
            for (int i = 0; i < config.num_instances; i++) {
                instances.push_back(generator.generate_rectangles(config.num_rectangles));
            }

            // Test ALL solvers on EACH instance before moving to next instance
            for (int instance_id = 0; instance_id < config.num_instances; instance_id++) {
                std::cout << "\n  Instance " << (instance_id + 1) << "/" << config.num_instances << ":" << std::endl;
                txt_file << "--- Instance " << instance_id << " ---\n";

                // Clear previous best markers for this instance
                for (auto& result : results) {
                    if (result.config_name == config.to_string() && result.instance_id == instance_id) {
                        result.is_best = false;
                    }
                }

                // Geometry-Based Solver
                std::cout << "    Geometry-Based Solver..." << std::flush;
                test_solver_on_instance("GeometryBased", instances[instance_id], config, instance_id);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Rule-Based (Permutation) Solver
                std::cout << "    Rule-Based Solver..." << std::flush;
                test_solver_on_instance("RuleBased", instances[instance_id], config, instance_id);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Relaxed Geometry Solver
                std::cout << "    Relaxed Geometry Solver..." << std::flush;
                test_solver_on_instance("RelaxedGeometry", instances[instance_id], config, instance_id);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Greedy Solver - Biggest First (Strategy 0)
                std::cout << "    Greedy (Biggest First)..." << std::flush;
                test_greedy_solver_on_instance("Greedy_Biggest", instances[instance_id], config, instance_id, 0);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Greedy Solver - Smallest First (Strategy 1)
                std::cout << "    Greedy (Smallest First)..." << std::flush;
                test_greedy_solver_on_instance("Greedy_Smallest", instances[instance_id], config, instance_id, 1);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Greedy Solver - Area Descending (Strategy 2)
                std::cout << "    Greedy (Area Descending)..." << std::flush;
                test_greedy_solver_on_instance("Greedy_AreaDesc", instances[instance_id], config, instance_id, 2);
                std::cout << " Done (" << results.back().cpu_time_seconds << "s)" << std::endl;

                // Find and mark the best solver for this instance
                find_best_solver_for_instance(config.to_string(), instance_id);

                // Show best solver for this instance
                std::cout << "    Best for this instance: ";
                for (const auto& result : results) {
                    if (result.config_name == config.to_string() &&
                        result.instance_id == instance_id &&
                        result.is_best) {
                        std::cout << result.solver_name
                                  << " (Boxes: " << result.num_boxes_used
                                  << ", Obj: " << result.objective_value << ")";
                        break;
                    }
                }
                std::cout << std::endl;
            }
        }

        std::cout << "\n=== Benchmark Complete ===\n" << std::endl;
        generate_comparison_table();
    }

    void test_solver_on_instance(const std::string& solver_name,
                                const std::vector<RectanglePlacement>& instance,
                                const TestConfig& config,
                                int instance_id) {

        RectangleFittingProblem problem(config.box_length, instance);
        BenchmarkResult result;
        result.solver_name = solver_name;
        result.config_name = config.to_string();
        result.instance_id = instance_id;

        std::vector<RectanglePlacement> solution;

        if (solver_name == "GeometryBased") {
            GeometryBasedNeighborhoodSolver solver;
            result.cpu_time_seconds = measure_cpu_time([&]() {
                solution = solver.solve(problem, 1, instance.size(), 2000);
            });
        }
        else if (solver_name == "RuleBased") {
            RuleBasedNeighborhoodSolver solver;
            result.cpu_time_seconds = measure_cpu_time([&]() {
                solution = solver.solve(problem, 1, instance.size(), 2000);
            });
        }
        else if (solver_name == "RelaxedGeometry") {
            RelaxedGeometryBasedNeighborhoodSolver solver;
            result.cpu_time_seconds = measure_cpu_time([&]() {
                solution = solver.solve(problem, 1, instance.size(), 2000);
            });
        }

        result.objective_value = problem.objective(solution);
        analyze_solution(solution, config.box_length, result);

        results.push_back(result);
        write_detailed_result(result);
    }

    void test_greedy_solver_on_instance(const std::string& solver_name,
                                       const std::vector<RectanglePlacement>& instance,
                                       const TestConfig& config,
                                       int instance_id,
                                       int strategy) {

        RectangleFittingProblem problem(config.box_length, instance);
        BenchmarkResult result;
        result.solver_name = solver_name;
        result.config_name = config.to_string();
        result.instance_id = instance_id;

        GreedySolver solver;
        solver.set_selection_strategy(strategy);

        std::vector<RectanglePlacement> solution;
        result.cpu_time_seconds = measure_cpu_time([&]() {
            solution = solver.solve(problem, 1, instance.size(), 2000);
        });

        result.objective_value = problem.objective(solution);
        analyze_solution(solution, config.box_length, result);

        results.push_back(result);
        write_detailed_result(result);
    }

    void write_detailed_result(const BenchmarkResult& result) {
        // Format: Solver Config Inst Boxes:Obj Time Feasible
        txt_file << std::left << std::setw(20) << result.solver_name
                << std::setw(15) << result.config_name
                << "Inst" << std::setw(3) << result.instance_id
                << "Boxes:" << std::setw(4) << result.num_boxes_used
                << " Obj:" << std::setw(8) << result.objective_value
                << " Time:" << std::fixed << std::setprecision(3) << std::setw(8) << result.cpu_time_seconds << "s"
                << " Feas:" << std::setw(5) << (result.is_feasible ? "Yes" : "No")
                << " Ovl:" << std::setw(3) << result.overlaps
                << " OOB:" << std::setw(3) << result.out_of_bounds;

        // Add "<= BEST" marker if this is the best solution for this instance
        if (result.is_best) {
            txt_file << "  <= BEST";
        }

        txt_file << "\n";
        txt_file.flush();
    }

    void generate_comparison_table() {
        txt_file << "\n========================================\n";
        txt_file << "COMPARISON TABLE (Average Results)\n";
        txt_file << "========================================\n\n";

        // Group results by solver
        std::map<std::string, std::vector<BenchmarkResult>> by_solver;
        for (const auto& result : results) {
            by_solver[result.solver_name].push_back(result);
        }

        // Table header
        txt_file << std::left << std::setw(20) << "Solver"
                << std::setw(10) << "Avg Boxes"
                << std::setw(12) << "Avg Obj"
                << std::setw(10) << "Avg Time"
                << std::setw(12) << "Feasible %"
                << std::setw(10) << "Best %"
                << std::setw(10) << "Avg Ovl"
                << std::setw(10) << "Avg OOB"
                << "\n";
        txt_file << std::string(94, '-') << "\n";

        // Calculate statistics for each solver
        for (const auto& [solver, solver_results] : by_solver) {
            std::vector<int> objectives;
            std::vector<double> times;
            std::vector<int> boxes;
            int feasible_count = 0;
            int best_count = 0;
            int total_overlaps = 0;
            int total_oob = 0;

            for (const auto& r : solver_results) {
                objectives.push_back(r.objective_value);
                times.push_back(r.cpu_time_seconds);
                boxes.push_back(r.num_boxes_used);
                if (r.is_feasible) feasible_count++;
                if (r.is_best) best_count++;
                total_overlaps += r.overlaps;
                total_oob += r.out_of_bounds;
            }

            double avg_obj = std::accumulate(objectives.begin(), objectives.end(), 0.0) / objectives.size();
            double avg_time = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
            double avg_boxes = std::accumulate(boxes.begin(), boxes.end(), 0.0) / boxes.size();
            double feasible_pct = 100.0 * feasible_count / solver_results.size();
            double best_pct = 100.0 * best_count / solver_results.size();
            double avg_overlaps = static_cast<double>(total_overlaps) / solver_results.size();
            double avg_oob = static_cast<double>(total_oob) / solver_results.size();

            txt_file << std::left << std::setw(20) << solver
                    << std::setw(10) << std::fixed << std::setprecision(2) << avg_boxes
                    << std::setw(12) << std::setprecision(2) << avg_obj
                    << std::setw(10) << std::setprecision(3) << avg_time
                    << std::setw(12) << std::setprecision(1) << feasible_pct << "%"
                    << std::setw(10) << std::setprecision(1) << best_pct << "%"
                    << std::setw(10) << std::setprecision(2) << avg_overlaps
                    << std::setw(10) << std::setprecision(2) << avg_oob
                    << "\n";
        }

        // Find overall best solver (most "BEST" markers)
        std::string overall_best_solver;
        int max_best_count = 0;
        for (const auto& [solver, solver_results] : by_solver) {
            int best_count = 0;
            for (const auto& r : solver_results) {
                if (r.is_best) best_count++;
            }
            if (best_count > max_best_count) {
                max_best_count = best_count;
                overall_best_solver = solver;
            }
        }

        txt_file << "\n========================================\n";
        txt_file << "OVERALL BEST SOLVER\n";
        txt_file << "========================================\n\n";
        txt_file << "Based on minimum boxes (objective as tie-breaker):\n";
        txt_file << "  " << overall_best_solver << " won " << max_best_count
                << " out of " << results.size() / by_solver.size() << " instances\n";

        txt_file << "\n========================================\n";
        txt_file << "PERFORMANCE NOTES\n";
        txt_file << "========================================\n\n";
        txt_file << "Benchmark completed at: " << get_current_time() << "\n";
        txt_file << "Total tests run: " << results.size() << "\n";
        txt_file << "Total instances: " << results.size() / by_solver.size() << "\n";
        txt_file << "Total solvers tested: " << by_solver.size() << "\n\n";
        txt_file << "NOTE: The benchmark runs multiple solvers on each instance,\n";
        txt_file << "so total time = (instance_count × solver_count × average_solve_time)\n";
        txt_file << "For 3 instances × 6 solvers × 15s each = ~270 seconds\n";

        txt_file.flush();

        std::cout << "\n✓ Results written to benchmark_results.txt" << std::endl;
        std::cout << "✓ Overall best solver: " << overall_best_solver << std::endl;
        std::cout << "✓ Check the file for detailed results and comparison table." << std::endl;
    }

    std::string get_current_time() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
};

// Enhanced test case definition
struct EnhancedTestCase {
    int numInstances;
    int numRectangles;
    int minSide1;
    int minSide2;
    int maxSide1;
    int maxSide2;
    int boxLength;
    std::string name;

    // Solver parameters
    int maxNeighborhoods;
    int numReruns;
    int maxRectInSubproblem;

    EnhancedTestCase(int ni, int nr, int min1, int min2, int max1, int max2, int bl,
                    const std::string& n = "", int mn = 100, int nruns = 5, int mrs = 50)
        : numInstances(ni), numRectangles(nr), minSide1(min1), minSide2(min2),
          maxSide1(max1), maxSide2(max2), boxLength(bl), name(n),
          maxNeighborhoods(mn), numReruns(nruns), maxRectInSubproblem(mrs) {}

    std::string toString() const {
        std::stringstream ss;
        ss << "Test: " << name
           << "\n  Instances: " << numInstances
           << ", Rectangles: " << numRectangles
           << "\n  Sides: [" << minSide1 << "," << minSide2 << "] to ["
           << maxSide1 << "," << maxSide2 << "]"
           << ", Box: " << boxLength
           << "\n  Solver params: MaxNeighbors=" << maxNeighborhoods
           << ", Reruns=" << numReruns
           << ", MaxRectSub=" << maxRectInSubproblem;
        return ss.str();
    }
};

struct EnhancedTestResult {
    std::string solverName;
    std::string testCaseName;
    int instanceId;
    int objectiveScore;
    int boxesUsed;
    double cpuTimeSeconds;
    bool solutionValid;

    // Additional metrics
    double avgOccupancy;
    int maxNeighborhoodsUsed;
    int numRerunsPerformed;
    int maxRectInSubproblem;
    int totalNeighborsGenerated;

    std::string toCSV() const {
        std::stringstream ss;
        ss << std::quoted(solverName) << ","
           << std::quoted(testCaseName) << ","
           << instanceId << ","
           << objectiveScore << ","
           << boxesUsed << ","
           << std::fixed << std::setprecision(6) << cpuTimeSeconds << ","
           << (solutionValid ? "VALID" : "INVALID") << ","
           << std::fixed << std::setprecision(2) << avgOccupancy << ","
           << maxNeighborhoodsUsed << ","
           << numRerunsPerformed << ","
           << maxRectInSubproblem << ","
           << totalNeighborsGenerated;
        return ss.str();
    }

    static std::string getCSVHeader() {
        return "Solver,TestCase,InstanceId,Score,Boxes,Time(s),Valid,AvgOccupancy%,MaxNeighbors,Reruns,MaxRectSub,TotalNeighbors";
    }

    void printDetails() const {
        std::cout << "  Solver: " << solverName << "\n"
                  << "  Score: " << objectiveScore << "\n"
                  << "  Boxes: " << boxesUsed << "\n"
                  << "  Time: " << std::fixed << std::setprecision(3) << cpuTimeSeconds << "s\n"
                  << "  Avg Occupancy: " << std::setprecision(1) << avgOccupancy << "%\n"
                  << "  Max Neighborhoods: " << maxNeighborhoodsUsed << "\n"
                  << "  Reruns: " << numRerunsPerformed << "\n"
                  << "  Max Rect in Subproblem: " << maxRectInSubproblem << "\n"
                  << "  Total Neighbors Generated: " << totalNeighborsGenerated << "\n"
                  << "  Valid: " << (solutionValid ? "YES" : "NO") << "\n";
    }
};

class EnhancedTestEnvironment {
private:
    std::vector<std::pair<std::string, std::unique_ptr<RectangleFittingProblemSolver>>> solvers;
    std::vector<EnhancedTestCase> testCases;
    std::mt19937 rng;
    int totalNeighborsCounter; // Track neighbors across runs

public:
    EnhancedTestEnvironment(int seed = 42) : rng(seed), totalNeighborsCounter(0) {}

    void addSolver(const std::string& name, std::unique_ptr<RectangleFittingProblemSolver> solver) {
        solvers.emplace_back(name, std::move(solver));
    }

    void addTestCase(const EnhancedTestCase& testCase) {
        testCases.push_back(testCase);
    }

    // Version 1: Quick test with small instances
    void setupQuickTest() {
        testCases.clear();

        // Small test cases that complete in minutes
        testCases.emplace_back(3, 20, 2, 2, 10, 10, 15, "Quick_Small", 50, 3, 10);
        testCases.emplace_back(2, 50, 3, 3, 15, 15, 20, "Quick_Medium", 100, 3, 15);
        testCases.emplace_back(1, 100, 5, 5, 20, 20, 30, "Quick_Large", 150, 3, 20);

        std::cout << "Quick test setup complete. Expected runtime: 2-5 minutes.\n";
    }

    // Version 2: Comprehensive test with meaningful instances
    void setupComprehensiveTest() {
        testCases.clear();

        // Your specified test cases with varied parameters
        testCases.emplace_back(5, 500, 3, 3, 50, 50, 100, "Comp_500R_100L", 200, 5, 50);
        testCases.emplace_back(5, 1000, 3, 3, 50, 50, 100, "Comp_1000R_100L", 300, 5, 75);
        testCases.emplace_back(3, 1500, 3, 3, 50, 50, 100, "Comp_1500R_100L", 400, 5, 100);
        testCases.emplace_back(2, 2000, 3, 3, 50, 50, 100, "Comp_2000R_100L", 500, 5, 125);
        testCases.emplace_back(2, 2000, 3, 3, 50, 50, 200, "Comp_2000R_200L", 500, 5, 150);

        // Additional varied test cases with different parameters
        testCases.emplace_back(3, 300, 1, 1, 30, 30, 50, "Comp_Varied1", 150, 4, 30);
        testCases.emplace_back(2, 800, 5, 5, 40, 40, 80, "Comp_Varied2", 250, 4, 60);

        std::cout << "Comprehensive test setup complete. Expected runtime: 30-60 minutes.\n";
    }

    // Version 3: Parameter sensitivity analysis
    void setupParameterAnalysis() {
        testCases.clear();

        // Fixed problem, varying solver parameters
        int baseRectangles = 200;
        int baseBox = 50;

        // Vary max neighborhoods
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Neighbors_50", 50, 3, 30);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Neighbors_100", 100, 3, 30);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Neighbors_200", 200, 3, 30);

        // Vary reruns
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Reruns_2", 100, 2, 30);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Reruns_5", 100, 5, 30);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Reruns_10", 100, 10, 30);

        // Vary max rectangles in subproblem
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Subprob_20", 100, 3, 20);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Subprob_50", 100, 3, 50);
        testCases.emplace_back(3, baseRectangles, 3, 3, 20, 20, baseBox,
                              "Param_Subprob_100", 100, 3, 100);

        std::cout << "Parameter analysis setup complete.\n";
    }

    void runTests(const std::string& outputFilename = "enhanced_test_results.csv") {
        std::ofstream resultsFile(outputFilename);
        if (!resultsFile.is_open()) {
            throw std::runtime_error("Could not open results file: " + outputFilename);
        }

        // Write CSV header
        resultsFile << EnhancedTestResult::getCSVHeader() << "\n";

        std::cout << "\n" << std::string(100, '=') << std::endl;
        std::cout << "ENHANCED TEST ENVIRONMENT" << std::endl;
        std::cout << "Solvers: " << solvers.size() << ", Test Cases: " << testCases.size() << std::endl;
        std::cout << std::string(100, '=') << "\n" << std::endl;

        int totalInstances = 0;
        for (const auto& tc : testCases) {
            totalInstances += tc.numInstances * solvers.size();
        }

        std::cout << "Total test runs: " << totalInstances << std::endl;
        auto startTotal = std::chrono::high_resolution_clock::now();

        int currentRun = 0;

        for (const auto& testCase : testCases) {
            std::cout << "\n" << std::string(80, '=') << std::endl;
            std::cout << "TEST CASE: " << testCase.name << std::endl;
            std::cout << testCase.toString() << std::endl;
            std::cout << std::string(80, '=') << "\n" << std::endl;

            // Generate all instances for this test case
            std::vector<std::vector<RectanglePlacement>> instances;
            InstanceGenerator generator(testCase.boxLength,
                                       testCase.minSide1, testCase.maxSide1,
                                       testCase.minSide2, testCase.maxSide2);

            for (int i = 0; i < testCase.numInstances; i++) {
                auto rectangles = generator.generate_rectangles(testCase.numRectangles);
                // Create initial solution using your generator's method
                auto instance = generator.create_better_initial_solution(rectangles, testCase.boxLength);
                instances.push_back(instance);
            }

            // Test each solver on each instance
            for (int instanceId = 0; instanceId < testCase.numInstances; instanceId++) {
                std::cout << "  Instance " << (instanceId + 1) << "/" << testCase.numInstances
                          << " (" << testCase.numRectangles << " rectangles):\n";

                for (const auto& [solverName, solver] : solvers) {
                    currentRun++;
                    std::cout << "    [" << currentRun << "/" << totalInstances << "] "
                              << solverName << "...\n";

                    // Reset neighbor counter for this run
                    totalNeighborsCounter = 0;

                    // Create fresh problem instance
                    RectangleFittingProblem problem(testCase.boxLength, instances[instanceId]);

                    // Run solver with test case parameters
                    auto startTime = std::chrono::high_resolution_clock::now();

                    // Wrap solver call to capture parameters
                    auto solution = runSolverWithMetrics(solver.get(), problem,
                                                        testCase.numReruns,
                                                        testCase.maxRectInSubproblem,
                                                        1000); // Default T

                    auto endTime = std::chrono::high_resolution_clock::now();
                    double elapsedSeconds = std::chrono::duration<double>(endTime - startTime).count();

                    // Calculate enhanced metrics
                    int objectiveScore = problem.objective(solution, 0);
                    int boxesUsed = countBoxesUsed(solution);
                    bool isValid = problem.solution_legal(solution);
                    double avgOccupancy = calculateAverageOccupancy(solution, testCase.boxLength);

                    // Store result
                    EnhancedTestResult result{
                        solverName,
                        testCase.name,
                        instanceId + 1,
                        objectiveScore,
                        boxesUsed,
                        elapsedSeconds,
                        isValid,
                        avgOccupancy,
                        testCase.maxNeighborhoods, // Could be dynamic in future
                        testCase.numReruns,
                        testCase.maxRectInSubproblem,
                        totalNeighborsCounter
                    };

                    // Write to CSV
                    resultsFile << result.toCSV() << "\n";
                    resultsFile.flush();

                    // Print to console
                    std::cout << "      Score: " << objectiveScore
                              << ", Boxes: " << boxesUsed
                              << ", Time: " << std::fixed << std::setprecision(3) << elapsedSeconds << "s"
                              << ", Occupancy: " << std::setprecision(1) << avgOccupancy << "%\n";

                    if (!isValid) {
                        std::cout << "      [WARNING: INVALID SOLUTION!]\n";
                    }
                }
            }
        }

        auto endTotal = std::chrono::high_resolution_clock::now();
        double totalSeconds = std::chrono::duration<double>(endTotal - startTotal).count();

        std::cout << "\n" << std::string(100, '=') << std::endl;
        std::cout << "TEST ENVIRONMENT COMPLETE" << std::endl;
        std::cout << "Total time: " << std::fixed << std::setprecision(2) << totalSeconds << " seconds" << std::endl;
        std::cout << "Results saved to: " << outputFilename << std::endl;
        std::cout << std::string(100, '=') << "\n" << std::endl;

        // Generate enhanced summary report
        generateEnhancedSummaryReport(outputFilename);

        resultsFile.close();
    }

private:
    std::vector<RectanglePlacement> runSolverWithMetrics(
        RectangleFittingProblemSolver* solver,
        RectangleFittingProblem& problem,
        int numReruns,
        int maxRectInSubproblem,
        int T) {

        // This is a wrapper to capture metrics during solving
        // In a real implementation, you might need to modify solvers to expose these metrics
        return solver->solve(problem, numReruns, maxRectInSubproblem, T);
    }

    int countBoxesUsed(const std::vector<RectanglePlacement>& solution) {
        std::unordered_set<int> boxes;
        for (const auto& rect : solution) {
            boxes.insert(rect.box_id);
        }
        return boxes.size();
    }

    double calculateAverageOccupancy(const std::vector<RectanglePlacement>& solution, int boxLength) {
        if (solution.empty()) return 0.0;

        std::unordered_map<int, long long> boxArea;
        long long boxCapacity = (long long)boxLength * boxLength;

        for (const auto& rect : solution) {
            boxArea[rect.box_id] += (long long)rect.width * rect.height;
        }

        double totalOccupancy = 0.0;
        for (const auto& [boxId, area] : boxArea) {
            double occupancy = (double)area / boxCapacity * 100.0;
            totalOccupancy += occupancy;
        }

        return boxArea.empty() ? 0.0 : totalOccupancy / boxArea.size();
    }

    void generateEnhancedSummaryReport(const std::string& resultsFile) {
        std::ifstream input(resultsFile);
        std::ofstream summary("enhanced_test_summary.txt");
        std::ofstream detailed("detailed_analysis.csv");

        if (!input.is_open() || !summary.is_open() || !detailed.is_open()) {
            std::cerr << "Could not generate summary reports" << std::endl;
            return;
        }

        // Parse results
        std::vector<EnhancedTestResult> allResults;
        std::string line;
        std::getline(input, line); // Skip header

        while (std::getline(input, line)) {
            std::stringstream ss(line);
            std::string solver, testCase, validStr;
            int instanceId, score, boxes, maxNeighbors, reruns, maxRectSub, totalNeighbors;
            double time, avgOccupancy;

            // Parse CSV
            std::getline(ss, solver, ',');
            solver = solver.substr(1, solver.length() - 2);

            std::getline(ss, testCase, ',');
            testCase = testCase.substr(1, testCase.length() - 2);

            ss >> instanceId; ss.ignore();
            ss >> score; ss.ignore();
            ss >> boxes; ss.ignore();
            ss >> time; ss.ignore();
            std::getline(ss, validStr, ',');
            ss >> avgOccupancy; ss.ignore();
            ss >> maxNeighbors; ss.ignore();
            ss >> reruns; ss.ignore();
            ss >> maxRectSub; ss.ignore();
            ss >> totalNeighbors;

            EnhancedTestResult result{
                solver, testCase, instanceId, score, boxes, time,
                (validStr == "VALID"), avgOccupancy, maxNeighbors,
                reruns, maxRectSub, totalNeighbors
            };

            allResults.push_back(result);
        }

        // Write comprehensive summary
        summary << "ENHANCED TEST ENVIRONMENT SUMMARY REPORT\n";
        summary << "=========================================\n\n";
        summary << "Generated: " << getCurrentTimestamp() << "\n";
        summary << "Total test runs: " << allResults.size() << "\n\n";

        // 1. Overall statistics by solver
        summary << "1. OVERALL SOLVER PERFORMANCE\n";
        summary << std::string(70, '-') << "\n";
        summary << std::left << std::setw(20) << "Solver"
                << std::setw(12) << "Avg Score"
                << std::setw(10) << "Avg Boxes"
                << std::setw(12) << "Avg Time(s)"
                << std::setw(12) << "Avg Occupancy%"
                << std::setw(15) << "Valid %"
                << std::setw(12) << "Best Score" << "\n";
        summary << std::string(93, '-') << "\n";

        std::map<std::string, std::vector<EnhancedTestResult>> resultsBySolver;
        for (const auto& res : allResults) {
            resultsBySolver[res.solverName].push_back(res);
        }

        for (const auto& [solver, results] : resultsBySolver) {
            double avgScore = 0, avgTime = 0, avgBoxes = 0, avgOccupancy = 0;
            int validCount = 0, bestScore = INT_MIN;

            for (const auto& res : results) {
                avgScore += res.objectiveScore;
                avgTime += res.cpuTimeSeconds;
                avgBoxes += res.boxesUsed;
                avgOccupancy += res.avgOccupancy;
                if (res.solutionValid) validCount++;
                if (res.objectiveScore > bestScore) bestScore = res.objectiveScore;
            }

            avgScore /= results.size();
            avgTime /= results.size();
            avgBoxes /= results.size();
            avgOccupancy /= results.size();
            double validPercent = 100.0 * validCount / results.size();

            summary << std::left << std::setw(20) << solver
                    << std::setw(12) << std::fixed << std::setprecision(0) << avgScore
                    << std::setw(10) << std::setprecision(1) << avgBoxes
                    << std::setw(12) << std::setprecision(3) << avgTime
                    << std::setw(12) << std::setprecision(1) << avgOccupancy << "%"
                    << std::setw(15) << std::setprecision(1) << validPercent << "%"
                    << std::setw(12) << std::setprecision(0) << bestScore << "\n";
        }

        // 2. Parameter sensitivity analysis
        summary << "\n\n2. PARAMETER SENSITIVITY ANALYSIS\n";
        summary << std::string(70, '-') << "\n";

        // Group by test case type
        std::map<std::string, std::vector<EnhancedTestResult>> resultsByTestType;
        for (const auto& res : allResults) {
            resultsByTestType[res.testCaseName].push_back(res);
        }

        // Detailed analysis for CSV
        detailed << "Solver,TestCase,ParamType,ParamValue,AvgScore,AvgTime,AvgOccupancy\n";

        for (const auto& [testType, results] : resultsByTestType) {
            // Extract parameter from test case name
            if (testType.find("Param_") == 0) {
                summary << "\nTest Type: " << testType << "\n";

                // Group by solver and parameter value
                std::map<std::string, std::map<int, std::vector<EnhancedTestResult>>> solverParamResults;

                for (const auto& res : results) {
                    int paramValue = 0;
                    if (testType.find("Neighbors") != std::string::npos) {
                        paramValue = res.maxNeighborhoodsUsed;
                    } else if (testType.find("Reruns") != std::string::npos) {
                        paramValue = res.numRerunsPerformed;
                    } else if (testType.find("Subprob") != std::string::npos) {
                        paramValue = res.maxRectInSubproblem;
                    }

                    solverParamResults[res.solverName][paramValue].push_back(res);
                }

                for (const auto& [solver, paramMap] : solverParamResults) {
                    summary << "  " << solver << ":\n";
                    for (const auto& [paramValue, paramResults] : paramMap) {
                        double avgScore = 0, avgTime = 0, avgOccupancy = 0;
                        for (const auto& res : paramResults) {
                            avgScore += res.objectiveScore;
                            avgTime += res.cpuTimeSeconds;
                            avgOccupancy += res.avgOccupancy;
                        }
                        avgScore /= paramResults.size();
                        avgTime /= paramResults.size();
                        avgOccupancy /= paramResults.size();

                        summary << "    Param=" << paramValue
                                << ": Score=" << std::fixed << std::setprecision(0) << avgScore
                                << ", Time=" << std::setprecision(3) << avgTime << "s"
                                << ", Occupancy=" << std::setprecision(1) << avgOccupancy << "%\n";

                        // Write to detailed CSV
                        std::string paramType = "Unknown";
                        if (testType.find("Neighbors") != std::string::npos) paramType = "MaxNeighbors";
                        else if (testType.find("Reruns") != std::string::npos) paramType = "Reruns";
                        else if (testType.find("Subprob") != std::string::npos) paramType = "MaxRectSub";

                        detailed << solver << "," << testType << ","
                                << paramType << "," << paramValue << ","
                                << std::fixed << std::setprecision(0) << avgScore << ","
                                << std::setprecision(3) << avgTime << ","
                                << std::setprecision(1) << avgOccupancy << "\n";
                    }
                }
            }
        }

        // 3. Performance ranking with confidence intervals
        summary << "\n\n3. PERFORMANCE RANKING WITH STATISTICS\n";
        summary << std::string(70, '-') << "\n";

        std::vector<std::pair<std::string, std::pair<double, double>>> solverStats; // name, (mean, stddev)

        for (const auto& [solver, results] : resultsBySolver) {
            std::vector<int> scores;
            for (const auto& res : results) {
                scores.push_back(res.objectiveScore);
            }

            double mean = std::accumulate(scores.begin(), scores.end(), 0.0) / scores.size();
            double sq_sum = std::inner_product(scores.begin(), scores.end(), scores.begin(), 0.0);
            double stdev = std::sqrt(sq_sum / scores.size() - mean * mean);

            solverStats.emplace_back(solver, std::make_pair(mean, stdev));
        }

        std::sort(solverStats.begin(), solverStats.end(),
                 [](const auto& a, const auto& b) { return a.second.first > b.second.first; });

        summary << "Ranking by average score (mean ± std dev):\n";
        for (size_t i = 0; i < solverStats.size(); i++) {
            summary << "  " << (i + 1) << ". " << std::left << std::setw(20) << solverStats[i].first
                    << ": " << std::fixed << std::setprecision(0) << solverStats[i].second.first
                    << " ± " << std::setprecision(0) << solverStats[i].second.second << "\n";
        }

        // 4. Recommendations based on results
        summary << "\n\n4. RECOMMENDATIONS\n";
        summary << std::string(70, '-') << "\n";

        // Find best solver for different metrics
        auto bestByScore = *std::max_element(solverStats.begin(), solverStats.end(),
                                           [](const auto& a, const auto& b) {
                                               return a.second.first < b.second.first;
                                           });
        auto bestByTime = *std::min_element(resultsBySolver.begin(), resultsBySolver.end(),
                                          [](const auto& a, const auto& b) {
                                              double timeA = 0, timeB = 0;
                                              for (const auto& res : a.second) timeA += res.cpuTimeSeconds;
                                              for (const auto& res : b.second) timeB += res.cpuTimeSeconds;
                                              return timeA/a.second.size() < timeB/b.second.size();
                                          });

        summary << "Best overall solver (score): " << bestByScore.first
                << " (avg score: " << std::fixed << std::setprecision(0) << bestByScore.second.first << ")\n";
        summary << "Fastest solver: " << bestByTime.first << "\n";

        summary << "\nSuggested parameters for large problems:\n";
        summary << "  - Max neighborhoods: 300-500\n";
        summary << "  - Reruns: 3-5\n";
        summary << "  - Max rectangles in subproblem: 50-100\n";

        summary.close();
        detailed.close();

        std::cout << "Enhanced summary saved to enhanced_test_summary.txt\n";
        std::cout << "Detailed analysis saved to detailed_analysis.csv\n";
    }

    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
};

#endif // ENHANCED_TEST_ENVIRONMENT_H