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

        // 1. Calculate Utilization per Box
        std::unordered_map<int, double> box_util_map;
        long long box_capacity = (long long)L * L;

        for (const auto& r : current_solution) {
            box_util_map[r.box_id] += (double)(r.width * r.height);
        }

        // 2. Identify "Locked" boxes (High utilization) vs "Active" boxes
        // We lower the threshold slightly to ensure we don't lock moderately bad boxes
        double lock_threshold = 0.92;
        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;

        for (const auto& r : current_solution) {
            double util = box_util_map[r.box_id] / box_capacity;
            if (util >= lock_threshold)
                locked.push_back(r);
            else
                active.push_back(r);
        }

        // Ensure we have enough active rectangles to do meaningful work
        if (active.size() < std::max(3, (int)(current_solution.size() * 0.10))) {
             // If everything is locked but we aren't optimal, unlock the worst locked boxes
             // (Logic simplified: just return if mostly optimal, or unlock everything if stuck)
             if (active.empty()) return current_solution;
        }

        // 3. SOLVE or SPLIT
        std::vector<RectanglePlacement> optimized_active;

        if ((int)active.size() <= max_rectangle_in_subproblem) {
            // --- DIRECT OPTIMIZATION ---
            RectangleFittingProblem sub(L, active);

            // Run a few passes of local search
            // (Assumes apply_local_search runs your neighborhood logic)
            auto current_sub = sub.get_current_solution();
            for(int k=0; k<5; k++) {
                current_sub = apply_local_search(sub, current_sub);
                sub.set_current_solution(current_sub);
            }
            optimized_active = sub.get_current_solution();
        }
        else {
            // --- SMART SPLITTING (SORTED BY DENSITY) ---

            // Group active rects by box
            std::unordered_map<int, std::vector<RectanglePlacement>> rects_by_box;
            for (const auto& r : active) {
                rects_by_box[r.box_id].push_back(r);
            }

            // Create a list of boxes sorted by utilization (Ascending)
            // We want the emptiest boxes (15%, 20%) to be at the front
            std::vector<std::pair<double, int>> sorted_boxes;
            for (const auto& [bid, _] : rects_by_box) {
                double u = box_util_map[bid] / box_capacity;
                sorted_boxes.push_back({u, bid});
            }
            std::sort(sorted_boxes.begin(), sorted_boxes.end());

            // Distribute into groups
            // Group 1 gets the "trash" (lowest util boxes) so they can be merged.
            std::vector<RectanglePlacement> group1, group2;
            int count_g1 = 0;
            int target = active.size() / 2;

            for (const auto& pair : sorted_boxes) {
                int bid = pair.second;
                auto& box_content = rects_by_box[bid];

                if (count_g1 < target) {
                    group1.insert(group1.end(), box_content.begin(), box_content.end());
                    count_g1 += box_content.size();
                } else {
                    group2.insert(group2.end(), box_content.begin(), box_content.end());
                }
            }

            // Fallback for edge cases (e.g. one massive box vs many small ones)
            if (group1.empty() || group2.empty()) {
                int mid = active.size() / 2;
                group1 = std::vector<RectanglePlacement>(active.begin(), active.begin() + mid);
                group2 = std::vector<RectanglePlacement>(active.begin() + mid, active.end());
            }

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
        reset_move_ids(optimized_active);
        auto merged = locked;
        merged.insert(merged.end(), optimized_active.begin(), optimized_active.end());
        reset_move_ids(merged);

        return merged;
    }

    // Helper for GUI stepping
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
                                                       std::vector<RectanglePlacement> initial) {
        auto neighbors = construct_neighbors(problem);
        if (neighbors.empty()) return initial;

        int current_obj = problem.objective(initial);
        std::vector<RectanglePlacement> best_sol = initial;
        int best_obj = current_obj;

        for (auto &n : neighbors) {
            int obj = problem.objective(n);
            if (obj > best_obj) {
                best_obj = obj;
                best_sol = n;
            }
        }

        if (best_obj > current_obj) {
            problem.set_current_solution(best_sol);
            return best_sol;
        }
        return initial;
    }




};

#endif //OPTIMALGO_SOLVER_H