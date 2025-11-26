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

    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem,
                                          int num_reruns,
                                          int max_rectangle_in_subproblem) override {
        return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
    }

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem,
                                                      int num_reruns,
                                                      int max_rectangle_in_subproblem) override {

        std::vector<RectanglePlacement> current_solution = problem.get_current_solution();

        if (num_reruns < 0) return current_solution;

        std::unordered_set<int> unique_boxes;
        for (const auto& p : current_solution) unique_boxes.insert(p.box_id);

        int num_boxes = unique_boxes.size();
        int num_rectangles = current_solution.size();

        bool is_small_subproblem = (num_boxes <= 2) || (num_rectangles < max_rectangle_in_subproblem);

        std::vector<RectanglePlacement> intermediate_result;

        // --- PHASE 1: Generate Solution ---
        if (is_small_subproblem) {
            intermediate_result = apply_local_search(problem, current_solution);
        }
        else {
            auto [left, right] = split_rectangles_by_box_id(current_solution);
            int L = problem.get_box_length();

            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            intermediate_result = safe_merge(solved_left, solved_right);
        }

        // --- PHASE 2: Optimization Rerun ---
        if (num_reruns > 0) {
            return filter_and_rerun(intermediate_result,
                                    problem.get_box_length(),
                                    num_reruns - 1,
                                    max_rectangle_in_subproblem);
        }

        return intermediate_result;
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