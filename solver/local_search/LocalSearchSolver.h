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


std::vector<RectanglePlacement>
solve_with_reruns(RectangleFittingProblem &problem,
                  int num_reruns,
                  int max_rectangle_in_subproblem) override
{
    std::vector<RectanglePlacement> current_solution = problem.get_current_solution();
    int L = problem.get_box_length();
    if (current_solution.empty()) return current_solution;

    // -----------------------------
    // 1. BOX UTILIZATION
    // -----------------------------
    std::unordered_map<int, double> box_areas;
    std::unordered_map<int, int> box_counts;

    for (const auto& r : current_solution) {
        box_areas[r.box_id] += (double)(r.width * r.height);
        box_counts[r.box_id]++;
    }

    double box_capacity = (double)L * L;

    // -----------------------------
    // 2. LOCK BOXES ≥ 80% UTIL
    //    BUT FORCE AT LEAST 2 ACTIVE BOXES
    // -----------------------------
    struct BoxInfo { int id; double util; };
    std::vector<BoxInfo> boxes;

    for (const auto& [bid, area] : box_areas)
        boxes.push_back({ bid, area / box_capacity });

    std::sort(boxes.begin(), boxes.end(),
              [](auto& a, auto& b) { return a.util > b.util; });

    std::unordered_set<int> locked_boxes;
    std::unordered_set<int> active_boxes;

    for (const auto& b : boxes) {
        if (b.util >= 0.80) locked_boxes.insert(b.id);
        else active_boxes.insert(b.id);
    }

    // ✅ FORCE AT LEAST 2 ACTIVE BOXES
    while ((int)active_boxes.size() < 2 && !locked_boxes.empty()) {
        int revive = *locked_boxes.begin();
        locked_boxes.erase(revive);
        active_boxes.insert(revive);
    }

    std::vector<RectanglePlacement> locked;
    std::vector<RectanglePlacement> active;

    for (const auto& r : current_solution) {
        if (locked_boxes.count(r.box_id)) locked.push_back(r);
        else active.push_back(r);
    }

    if (active.empty()) return current_solution;

    // -----------------------------
    // 3. SUBPROBLEM LIMIT BY RECT COUNT
    // -----------------------------
    std::vector<RectanglePlacement> optimized_active;

    if ((int)active.size() <= max_rectangle_in_subproblem) {
        // --- LOCAL SEARCH UNTIL TRUE LOCAL OPTIMUM ---
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
        // ✅ SPLIT BY BOX ID FIRST (STRUCTURAL)
        std::unordered_map<int, std::vector<RectanglePlacement>> by_box;
        for (const auto& r : active) by_box[r.box_id].push_back(r);

        if (by_box.size() > 1) {
            auto it = by_box.begin();
            std::vector<RectanglePlacement> left = it->second;
            ++it;

            std::vector<RectanglePlacement> right;
            for (; it != by_box.end(); ++it)
                right.insert(right.end(), it->second.begin(), it->second.end());

            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            auto solved_left  = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            optimized_active = safe_merge(solved_left, solved_right);
        }
        else {
            // ✅ FALLBACK: PURE RECT SPLIT
            int mid = active.size() / 2;
            std::vector<RectanglePlacement> left(active.begin(), active.begin() + mid);
            std::vector<RectanglePlacement> right(active.begin() + mid, active.end());

            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            auto solved_left  = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            optimized_active = safe_merge(solved_left, solved_right);
        }
    }

    // -----------------------------
    // 4. SAFE MERGE
    // -----------------------------
    return safe_merge(locked, optimized_active);
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