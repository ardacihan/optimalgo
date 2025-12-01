// Solver.h
// Base class for rectangle packing solvers with "Filter & Rerun" optimization
//

#ifndef OPTIMALGO_SOLVER_H
#define OPTIMALGO_SOLVER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>

#include "solver/RectangleFittingProblemSolver.h"

class LocalSearchSolver : public RectangleFittingProblemSolver {
public:
    virtual ~LocalSearchSolver() = default;

    std::vector<RectanglePlacement>
    solve(RectangleFittingProblem &problem,
          int num_reruns,
          int max_rectangle_in_subproblem) override
    {
        RectangleFittingProblem working = problem;

        for (int i = 0; i < num_reruns; ++i) {
            auto result = solve_with_reruns(working, num_reruns, max_rectangle_in_subproblem);
            std::cout << "Run" << i << std::endl;
            working.set_current_solution(result);
        }

        return working.get_current_solution();
    }
    void reset_move_ids(std::vector<RectanglePlacement>& sol) {
        for (auto& r : sol)
            r.move_id = -1;
    }



    std::vector<RectanglePlacement>
    solve_with_reruns(RectangleFittingProblem &problem,
                      int num_reruns,
                      int max_rectangle_in_subproblem) override
    {
        std::vector<RectanglePlacement> current_solution = problem.get_current_solution();
        int L = problem.get_box_length();
        if (current_solution.empty()) return current_solution;

        std::cout << "\n=== RERUN START ===" << std::endl;
        std::cout << "Total rectangles: " << current_solution.size() << std::endl;

        // 1. Calculate Utilization per Box
        std::unordered_map<int, double> box_util_map;
        std::unordered_map<int, int> box_rect_count;
        long long box_capacity = (long long)L * L;

        for (const auto& r : current_solution) {
            box_util_map[r.box_id] += (double)(r.width * r.height);
            box_rect_count[r.box_id]++;
        }

        // Print all boxes and their status
        std::cout << "All boxes:" << std::endl;
        for (const auto& [box_id, area] : box_util_map) {
            double util = area / box_capacity;
            std::cout << "  Box " << box_id << ": " << box_rect_count[box_id]
                      << " rectangles, " << (util * 100) << "% utilization"
                      << (util >= 0.85 ? " [LOCKED]" : " [ACTIVE]") << std::endl;
        }

        // 2. Identify "Locked" boxes (>85% utilization) vs "Active" boxes
        double lock_threshold = 0.85;
        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;
        std::set<int> active_box_ids;
        std::set<int> locked_box_ids;

        for (const auto& r : current_solution) {
            double util = box_util_map[r.box_id] / box_capacity;
            if (util >= lock_threshold) {
                locked.push_back(r);
                locked_box_ids.insert(r.box_id);
            } else {
                active.push_back(r);
                active_box_ids.insert(r.box_id);
            }
        }

        // Print active box IDs
        std::cout << "\nActive box IDs: ";
        if (active_box_ids.empty()) {
            std::cout << "NONE";
        } else {
            for (int box_id : active_box_ids) {
                std::cout << box_id << " ";
            }
        }
        std::cout << std::endl;

        std::cout << "Locked box IDs: ";
        if (locked_box_ids.empty()) {
            std::cout << "NONE";
        } else {
            for (int box_id : locked_box_ids) {
                std::cout << box_id << " ";
            }
        }
        std::cout << std::endl;

        std::cout << "Locked rectangles: " << locked.size()
                  << " (in " << locked_box_ids.size() << " boxes)" << std::endl;
        std::cout << "Active rectangles: " << active.size()
                  << " (in " << active_box_ids.size() << " boxes)" << std::endl;

        // If no active rectangles, return current solution
        if (active.empty()) {
            std::cout << "No active rectangles to optimize" << std::endl;
            return current_solution;
        }

        // 3. SOLVE or SPLIT based on max_rectangle_in_subproblem
        std::vector<RectanglePlacement> optimized_active;

        if ((int)active.size() <= max_rectangle_in_subproblem) {
            std::cout << "Solving " << active.size() << " active rectangles directly" << std::endl;

            RectangleFittingProblem sub(L, active);

            auto current_sub = sub.get_current_solution();
            for(int k = 0; k < 5; k++) {
                current_sub = apply_local_search(sub, current_sub);
                sub.set_current_solution(current_sub);
            }
            optimized_active = sub.get_current_solution();
        }
        else {
            std::cout << "Splitting " << active.size() << " active rectangles (exceeds limit of "
                      << max_rectangle_in_subproblem << ")" << std::endl;

            // Split into two groups
            int mid = active.size() / 2;
            std::vector<RectanglePlacement> group1(active.begin(), active.begin() + mid);
            std::vector<RectanglePlacement> group2(active.begin() + mid, active.end());

            std::cout << "Split into: " << group1.size() << " and " << group2.size() << " rectangles" << std::endl;

            // Recursive Solve
            RectangleFittingProblem p1(L, group1);
            RectangleFittingProblem p2(L, group2);

            auto s1 = solve_with_reruns(p1, std::max(1, num_reruns - 1), max_rectangle_in_subproblem);
            auto s2 = solve_with_reruns(p2, std::max(1, num_reruns - 1), max_rectangle_in_subproblem);

            // Merge results
            optimized_active = s1;
            optimized_active.insert(optimized_active.end(), s2.begin(), s2.end());
        }

        // 4. Final Merge
        std::cout << "\nMerging results..." << std::endl;

        // Give optimized active rectangles new box IDs to avoid conflicts with locked boxes
        int max_locked_box_id = -1;
        for (const auto& r : locked) {
            if (r.box_id > max_locked_box_id) {
                max_locked_box_id = r.box_id;
            }
        }

        std::cout << "Max locked box ID: " << max_locked_box_id << std::endl;
        std::cout << "Remapping active rectangle box IDs..." << std::endl;

        for (auto& r : optimized_active) {
            r.box_id += (max_locked_box_id + 1);
        }

        reset_move_ids(optimized_active);
        auto merged = locked;
        merged.insert(merged.end(), optimized_active.begin(), optimized_active.end());
        reset_move_ids(merged);

        // Verify rectangle count
        if (merged.size() != current_solution.size()) {
            std::cout << "ERROR: Lost rectangles! Original: " << current_solution.size()
                      << ", Merged: " << merged.size() << std::endl;
            return current_solution;
        }

        // Print final box count
        std::unordered_set<int> final_boxes;
        for (const auto& r : merged) {
            final_boxes.insert(r.box_id);
        }
        std::cout << "Final boxes: " << final_boxes.size() << std::endl;
        std::cout << "=== RERUN END ===\n" << std::endl;

        return merged;
    }


    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T = 1000) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();
        auto neighbors = construct_neighbors(problem);

        if (neighbors.empty()) return solution;

        int current_obj = (T < 1000) ? problem.objective(solution, T) : problem.objective(solution);
        std::vector<RectanglePlacement> best_neighbor = solution;
        int best_obj = current_obj;

        for (auto &n : neighbors) {
            int obj = (T < 1000) ? problem.objective(n, T) : problem.objective(n);
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
        RectangleFittingProblem &problem) = 0;

std::vector<RectanglePlacement> apply_local_search(RectangleFittingProblem &problem,
                                                   std::vector<RectanglePlacement> initial,
                                                   int max_iterations = 1000,
                                                   int max_non_improving = 2) {
    auto current_solution = initial;
    int current_obj = problem.objective(current_solution);

    std::vector<RectanglePlacement> best_sol = current_solution;
    int best_obj = current_obj;

    int non_improving_count = 0;

    for (int iter = 0; iter < max_iterations && non_improving_count < max_non_improving; iter++) {
        std::cout << "Local Search Iteration " << iter
                  << ", Current Objective: " << current_obj << std::endl;

        auto neighbors = construct_neighbors(problem);
        if (neighbors.empty()) break;

        bool improved = false;
        int best_neighbor_obj = current_obj;
        std::vector<RectanglePlacement> best_neighbor = current_solution;

        // Find best neighbor
        for (auto &n : neighbors) {
            int obj = problem.objective(n);
            if (obj > best_neighbor_obj) {
                best_neighbor_obj = obj;
                best_neighbor = n;
                improved = true;
            }
        }

        if (improved) {
            current_solution = best_neighbor;
            current_obj = best_neighbor_obj;

            if (current_obj > best_obj) {
                best_sol = current_solution;
                best_obj = current_obj;
                non_improving_count = 0; // Reset counter on improvement
                std::cout << "New best objective: " << best_obj << std::endl;
            }

            // Update problem state for next neighborhood construction
            problem.set_current_solution(current_solution);
        } else {
            non_improving_count++;
            std::cout << "No improvement found (" << non_improving_count
                      << "/" << max_non_improving << ")" << std::endl;

            // DON'T return early - continue to use up the non-improving count
            // This allows for some exploration even without immediate improvement
        }
    }

    std::cout << "Local search finished. Best objective: " << best_obj
              << " (improvement: " << (best_obj - problem.objective(initial))
              << ")" << std::endl;

    return best_sol;
}




};

#endif //OPTIMALGO_SOLVER_H