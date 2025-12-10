// ============================================================================
// FILE: benchmark.cpp - Comprehensive Solver Benchmark
// ============================================================================
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <map>
#include <algorithm>
#include <numeric>
#include <sstream>
#include "problem/RectangleFittingProblem.h"
#include "gui/InstanceGenerator.h"
#include "solver/local_search/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.h"
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
private:
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

public:
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

private:
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

int main2(int argc, char* argv[]) {
    bool quick_mode = true;

    std::cout << "\n========================================" << std::endl;
    std::cout << "RECTANGLE PACKING SOLVER BENCHMARK" << std::endl;
    std::cout << "========================================\n" << std::endl;

    std::cout << "NOTE: This benchmark tests 6 solvers on each instance:" << std::endl;
    std::cout << "  1. Geometry-Based Solver" << std::endl;
    std::cout << "  2. Rule-Based Solver" << std::endl;
    std::cout << "  3. Relaxed Geometry Solver" << std::endl;
    std::cout << "  4. Greedy (Biggest First)" << std::endl;
    std::cout << "  5. Greedy (Smallest First)" << std::endl;
    std::cout << "  6. Greedy (Area Descending)" << std::endl;
    std::cout << std::endl;

    // Use fixed output filename
    std::string output_filename = "benchmark_results.txt";
    BenchmarkRunner runner(output_filename);

    if (quick_mode) {
        std::cout << "=== QUICK BENCHMARK MODE ===\n" << std::endl;

        // Reduced for faster testing
        runner.add_config({1, 500, 10, 20, 10, 20, 80});
        runner.add_config({1, 500, 10, 20, 10, 20, 80});
        runner.add_config({1, 500, 10, 40, 10, 40, 80});
        runner.add_config({1, 500, 10, 40, 10, 40, 100});


        runner.add_config({1, 1000, 10, 20, 10, 20, 80});
        runner.add_config({1, 1000, 10, 20, 10, 20, 80});
        runner.add_config({1, 1000, 10, 40, 10, 40, 80});
        runner.add_config({1, 1000, 10, 40, 10, 40, 100});
        // runner.add_config({1, 200, 10, 20, 10, 20, 50}); // 1 instance

    } else {
        std::cout << "=== FULL BENCHMARK MODE ===\n" << std::endl;

        runner.add_config({3, 100, 5, 10, 5, 10, 20});
        runner.add_config({3, 200, 5, 15, 5, 15, 30});
        runner.add_config({2, 300, 10, 20, 10, 20, 50});
    }

    runner.run_benchmark();

    std::cout << "\n========================================" << std::endl;
    std::cout << "BENCHMARK COMPLETE" << std::endl;
    std::cout << "========================================\n" << std::endl;
    
    return 0;
}