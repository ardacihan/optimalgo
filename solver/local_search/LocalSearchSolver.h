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

        // -----------------------------
        // 1. COMPUTE BOX UTILIZATION
        // -----------------------------
        std::unordered_map<int, double> box_areas;
        for (const auto& r : current_solution)
            box_areas[r.box_id] += (double)(r.width * r.height);

        double box_capacity = (double)L * L;

        // -----------------------------
        // 2. FILTER MOST-OCCUPIED BOXES (>= 80%)
        // -----------------------------
        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;

        for (const auto& r : current_solution) {
            double util = box_areas[r.box_id] / box_capacity;
            if (util >= 0.80)
                locked.push_back(r);
            else
                active.push_back(r);
        }

        // ✅ Force at least 2 boxes active
        std::unordered_set<int> active_boxes;
        for (auto& r : active) active_boxes.insert(r.box_id);
        for (auto& r : locked) {
            if (active_boxes.size() >= 2) break;
            active.push_back(r);
            active_boxes.insert(r.box_id);
        }

        if (active.empty()) return current_solution;

        // -----------------------------
        // 3. SUBPROBLEM SOLVE OR SPLIT
        // -----------------------------
        std::vector<RectanglePlacement> optimized_active;

        if ((int)active.size() <= max_rectangle_in_subproblem) {
            RectangleFittingProblem sub(L, active);
            double prev_score = -1e18;

            while (true) {
                auto candidate = apply_local_search(sub, sub.get_current_solution());
                double score = sub.objective(candidate);
                if (score <= prev_score) break;

                prev_score = score;
                sub.set_current_solution(candidate);
            }

            optimized_active = sub.get_current_solution();
        }
        else {
            int mid = active.size() / 2;
            std::vector<RectanglePlacement> left(active.begin(), active.begin() + mid);
            std::vector<RectanglePlacement> right(active.begin() + mid, active.end());

            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            auto solved_left  = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            optimized_active = safe_merge(solved_left, solved_right);
        }

        // -----------------------------
        // 4. RESET MOVE IDS AFTER RERUN
        // -----------------------------
        reset_move_ids(optimized_active);

        // -----------------------------
        // 5. SAFE MERGE
        // -----------------------------
        auto merged = safe_merge(locked, optimized_active);

        // ✅ Also reset locked (critical)
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