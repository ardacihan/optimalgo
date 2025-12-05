// Solver.h
// Base class for rectangle packing solvers with "Filter & Rerun" optimization
// Refactored to properly implement iterative filter-solve-merge cycles

#ifndef OPTIMALGO_SOLVER_H
#define OPTIMALGO_SOLVER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>
#include <random>

#include "solver/RectangleFittingProblemSolver.h"

class LocalSearchSolver : public RectangleFittingProblemSolver {
private:
    std::random_device rd;
    std::mt19937 gen;

public:
    LocalSearchSolver() : gen(rd()) {}

    virtual ~LocalSearchSolver() = default;

    std::vector<RectanglePlacement>
    solve(RectangleFittingProblem &problem,
          int num_reruns,
          int max_rectangle_in_subproblem,
          int T) override
    {
        return solve(problem, num_reruns, max_rectangle_in_subproblem, T, 0.85);
    }

std::vector<RectanglePlacement>
solve(RectangleFittingProblem &problem,
      int num_reruns,
      int max_rectangle_in_subproblem,
      int T,
      double lock_threshold)
{
    auto current_solution = problem.get_current_solution();
    if (current_solution.empty()) return current_solution;

    int current_obj = problem.objective(current_solution, T);
    std::cout << "\n=== STARTING ITERATIVE RERUN PROCESS ===" << std::endl;
    std::cout << "Total rectangles: " << current_solution.size() << std::endl;
    std::cout << "Initial objective: " << current_obj << std::endl;
    std::cout << "Lock threshold: " << (lock_threshold * 100) << "%" << std::endl;
    std::cout << "Maximum reruns: " << num_reruns << std::endl;

    bool improved = true;
    int rerun_count = 0;
    int max_reruns = std::max(1, num_reruns); // Ensure at least 1 rerun

    // Keep rerunning while we're improving AND under the limit
    while (improved && rerun_count < max_reruns) {
        std::cout << "\n=== RERUN " << rerun_count << " ===" << std::endl;

        // Store previous solution for comparison
        auto previous_solution = current_solution;
        int previous_obj = current_obj;

        // 1. Filter: separate locked vs active boxes
        auto [locked, active] = filter_by_utilization(
            current_solution, problem.get_box_length(), lock_threshold);

        // If no active rectangles, we're done
        if (active.empty()) {
            std::cout << "No active rectangles remaining - stopping reruns" << std::endl;
            break;
        }

        // 2. Re-solve: optimize active rectangles
        auto optimized_active = solve_subproblem(
            active, problem.get_box_length(),
            max_rectangle_in_subproblem, T);

        // 3. Merge: combine locked + optimized
        current_solution = merge_solutions(locked, optimized_active);
        problem.set_current_solution(current_solution);

        // 4. Calculate new objective
        current_obj = problem.objective(current_solution, T);
        std::cout << "Objective after rerun " << rerun_count << ": " << current_obj
                  << " (previous: " << previous_obj << ")" << std::endl;

        // 5. Check if we improved
        if (current_obj > previous_obj) {
            std::cout << "✓ Improved by " << (current_obj - previous_obj) << std::endl;
            improved = true;
        } else {
            std::cout << "✗ No improvement - stopping reruns" << std::endl;
            improved = false;
            // Optional: revert to previous solution if no improvement
            // current_solution = previous_solution;
            // current_obj = previous_obj;
            // problem.set_current_solution(current_solution);
        }

        rerun_count++;
    }

    std::cout << "\n=== RERUN PROCESS COMPLETE ===" << std::endl;
    std::cout << "Total reruns performed: " << rerun_count << std::endl;
    std::cout << "Final objective: " << current_obj << std::endl;

    return current_solution;
}

    std::vector<RectanglePlacement>
    solve_with_reruns(RectangleFittingProblem &problem,
                      int num_reruns,
                      int max_rectangle_in_subproblem,
                      int T) override
    {
        // Legacy method - redirect to new solve
        return solve(problem, num_reruns, max_rectangle_in_subproblem, T);
    }

    void reset_move_ids(std::vector<RectanglePlacement>& sol) {
        for (auto& r : sol)
            r.move_id = -1;
    }

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T = 1000) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();
        auto neighbors = construct_neighbors(problem, T);

        if (neighbors.empty()) return solution;

        int current_obj = problem.objective(solution, T);
        std::vector<RectanglePlacement> best_neighbor = solution;
        int best_obj = current_obj;

        for (auto &n : neighbors) {
            int obj = problem.objective(n, T);
            if (obj > best_obj) {
                best_obj = obj;
                best_neighbor = n;
            }
        }

        if (best_obj > current_obj) {
            problem.set_current_solution(best_neighbor);
            return best_neighbor;
        }
        return solution;
    }

protected:
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) = 0;

    // Filter rectangles into locked (high utilization) and active (low utilization)
    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    filter_by_utilization(const std::vector<RectanglePlacement>& solution,
                         int box_length,
                         double lock_threshold)
    {
        // Calculate utilization per box
        std::unordered_map<int, long long> box_area;
        std::unordered_map<int, int> box_rect_count;
        long long box_capacity = (long long)box_length * box_length;

        for (const auto& r : solution) {
            box_area[r.box_id] += (long long)r.width * r.height;
            box_rect_count[r.box_id]++;
        }

        // Print all boxes and their status
        std::cout << "\nBox utilization analysis:" << std::endl;
        std::set<int> locked_box_ids;
        std::set<int> active_box_ids;

        for (const auto& [box_id, area] : box_area) {
            double util = (double)area / box_capacity;
            bool is_locked = (util >= lock_threshold);

            std::cout << "  Box " << box_id << ": " << box_rect_count[box_id]
                      << " rectangles, " << (util * 100.0) << "% utilization"
                      << (is_locked ? " [LOCKED]" : " [ACTIVE]") << std::endl;

            if (is_locked) {
                locked_box_ids.insert(box_id);
            } else {
                active_box_ids.insert(box_id);
            }
        }

        // Separate rectangles
        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;

        for (const auto& r : solution) {
            double util = (double)box_area[r.box_id] / box_capacity;
            if (util >= lock_threshold) {
                locked.push_back(r);
            } else {
                active.push_back(r);
            }
        }

        std::cout << "\nFiltering results:" << std::endl;
        std::cout << "  Locked: " << locked.size() << " rectangles in "
                  << locked_box_ids.size() << " boxes" << std::endl;
        std::cout << "  Active: " << active.size() << " rectangles in "
                  << active_box_ids.size() << " boxes" << std::endl;

        return {locked, active};
    }

    // Solve a subproblem (possibly by splitting if too large)
    std::vector<RectanglePlacement>
    solve_subproblem(const std::vector<RectanglePlacement>& rectangles,
                    int box_length,
                    int max_rectangle_in_subproblem,
                    int T)
    {
        std::cout << "\nSolving subproblem with " << rectangles.size() << " rectangles" << std::endl;

        // Case 1: Small enough to solve directly
        if ((int)rectangles.size() <= max_rectangle_in_subproblem) {
            std::cout << "  Solving directly (within limit of " << max_rectangle_in_subproblem << ")" << std::endl;

            RectangleFittingProblem sub(box_length, rectangles);
            auto solution = sub.get_current_solution();

            // Apply local search with proper cooling schedule
            // More iterations for larger subproblems
            int base_iterations = std::min(500, (int)rectangles.size() * 10);
            solution = apply_local_search(sub, solution, base_iterations, 5, T);

            return solution;
        }

        // Case 2: Too large - split and solve recursively
        std::cout << "  Splitting (exceeds limit of " << max_rectangle_in_subproblem << ")" << std::endl;

        auto [group1, group2] = split_rectangles(rectangles);

        std::cout << "  Split into: " << group1.size() << " and " << group2.size() << " rectangles" << std::endl;

        // Recursively solve each group
        auto solution1 = solve_subproblem(group1, box_length, max_rectangle_in_subproblem, T);
        auto solution2 = solve_subproblem(group2, box_length, max_rectangle_in_subproblem, T);

        // Merge the two solutions
        std::vector<RectanglePlacement> merged = solution1;
        merged.insert(merged.end(), solution2.begin(), solution2.end());

        return merged;
    }

    // Split rectangles into two groups (box-aware splitting)
    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    split_rectangles(const std::vector<RectanglePlacement>& rectangles)
    {
        // Group rectangles by box ID
        std::unordered_map<int, std::vector<RectanglePlacement>> box_groups;
        for (const auto& r : rectangles) {
            box_groups[r.box_id].push_back(r);
        }

        // If multiple boxes, split by box
        if (box_groups.size() > 1) {
            std::vector<int> box_ids;
            for (const auto& [box_id, _] : box_groups) {
                box_ids.push_back(box_id);
            }
            std::sort(box_ids.begin(), box_ids.end());

            int mid = box_ids.size() / 2;
            std::vector<RectanglePlacement> group1, group2;

            for (int i = 0; i < (int)box_ids.size(); ++i) {
                if (i < mid) {
                    group1.insert(group1.end(),
                                 box_groups[box_ids[i]].begin(),
                                 box_groups[box_ids[i]].end());
                } else {
                    group2.insert(group2.end(),
                                 box_groups[box_ids[i]].begin(),
                                 box_groups[box_ids[i]].end());
                }
            }

            return {group1, group2};
        }

        // Single box - split by size (large vs small)
        std::vector<RectanglePlacement> sorted = rectangles;
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
            return (a.width * a.height) > (b.width * b.height);
        });

        int mid = sorted.size() / 2;
        std::vector<RectanglePlacement> group1(sorted.begin(), sorted.begin() + mid);
        std::vector<RectanglePlacement> group2(sorted.begin() + mid, sorted.end());

        return {group1, group2};
    }

    // Merge locked and optimized solutions, normalizing box IDs
    std::vector<RectanglePlacement>
    merge_solutions(const std::vector<RectanglePlacement>& locked,
                   std::vector<RectanglePlacement> active)
    {
        std::cout << "\nMerging solutions..." << std::endl;

        // Find max box ID in locked rectangles
        int max_locked_box_id = -1;
        for (const auto& r : locked) {
            max_locked_box_id = std::max(max_locked_box_id, r.box_id);
        }

        std::cout << "  Max locked box ID: " << max_locked_box_id << std::endl;

        // Remap active box IDs to avoid conflicts
        std::unordered_map<int, int> box_id_mapping;
        int next_box_id = max_locked_box_id + 1;

        for (auto& r : active) {
            if (box_id_mapping.find(r.box_id) == box_id_mapping.end()) {
                box_id_mapping[r.box_id] = next_box_id++;
            }
            r.box_id = box_id_mapping[r.box_id];
        }

        std::cout << "  Remapped " << box_id_mapping.size() << " active box IDs" << std::endl;

        // Combine
        std::vector<RectanglePlacement> merged = locked;
        merged.insert(merged.end(), active.begin(), active.end());

        // Reset move IDs
        reset_move_ids(merged);

        // Count final boxes
        std::unordered_set<int> final_boxes;
        for (const auto& r : merged) {
            final_boxes.insert(r.box_id);
        }

        std::cout << "  Final solution: " << merged.size() << " rectangles in "
                  << final_boxes.size() << " boxes" << std::endl;

        return merged;
    }

    std::vector<RectanglePlacement> apply_local_search(
        RectangleFittingProblem &problem,
        std::vector<RectanglePlacement> initial,
        int max_iterations = 2000,
        int max_non_improving = 5,
        int T = 1000)
    {
        auto current_solution = initial;
        int current_obj = problem.objective(current_solution, T);

        std::vector<RectanglePlacement> best_sol = current_solution;
        int best_obj = current_obj;

        int non_improving_count = 0;
        int iteration = 0;

        std::cout << "    Starting local search (initial obj: " << current_obj << ")" << std::endl;

        // Simulated annealing parameters
        double start_temperature = T * 2.0; // Start hotter
        double current_temperature = start_temperature;
        double min_temperature = 1.0;
        double cooling_rate = 0.95;

        std::uniform_real_distribution<> dist(0.0, 1.0);

        while (non_improving_count < max_non_improving) {
            iteration++;

            // Update temperature (exponential cooling)
            current_temperature = start_temperature * std::pow(cooling_rate, iteration);
            current_temperature = std::max(min_temperature, current_temperature);

            int int_temperature = (int)current_temperature;

            // Generate neighbors
            auto neighbors = construct_neighbors(problem, int_temperature);

            if (neighbors.empty()) {
                std::cout << "    No neighbors at iteration " << iteration << std::endl;
                break;
            }

            // Find best neighbor
            int best_neighbor_obj = current_obj;
            std::vector<RectanglePlacement> best_neighbor = current_solution;

            for (auto &n : neighbors) {
                int obj = problem.objective(n, int_temperature);
                if (obj > best_neighbor_obj) {
                    best_neighbor_obj = obj;
                    best_neighbor = n;
                }
            }

            // Decide whether to accept the move
            bool accepted = false;

            if (best_neighbor_obj > current_obj) {
                // Always accept improving moves
                accepted = true;
            }

            if (accepted) {
                current_solution = best_neighbor;
                current_obj = best_neighbor_obj;

                if (current_obj > best_obj) {
                    best_sol = current_solution;
                    best_obj = current_obj;
                    non_improving_count = 0;
                    std::cout << "    Iteration " << iteration << " (T=" << int_temperature
                              << "): new best = " << best_obj << std::endl;
                } else {
                    non_improving_count++;
                }

                problem.set_current_solution(current_solution);
            } else {
                non_improving_count++;
            }
        }

        int improvement = best_obj - problem.objective(initial, T);
        std::cout << "    Local search finished after " << iteration
                  << " iterations (best: " << best_obj
                  << ", improvement: " << improvement
                  << ", final T: " << (int)current_temperature << ")" << std::endl;

        return best_sol;
    }
};

#endif //OPTIMALGO_SOLVER_H