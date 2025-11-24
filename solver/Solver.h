//
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
#include <limits>
#include <cmath>

// Forward declaration assuming this class exists in your project
// class RectangleFittingProblem;
// struct RectanglePlacement;

class Solver {
public:
    virtual ~Solver() = default;

    /**
     * Main entry point.
     * Delegates to the recursive logic.
     */
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem,
                                          int num_reruns,
                                          int max_rectangle_in_subproblem) {
        return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
    }

    /**
     * Recursive Divide-and-Conquer Solver with Optimization Reruns.
     * * Logic Flow:
     * 1. Check if problem is small (Leaf) or large (Node).
     * 2. If Leaf: Run Local Search (Hill Climbing).
     * 3. If Node: Split problem, recurse left/right, merge.
     * 4. Post-Process: "Filter & Rerun"
     * - Identify boxes with low utilization.
     * - 'Freeze' good boxes.
     * - 'Melt' bad boxes and solve them again as a fresh group.
     */
    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem,
                                                      int num_reruns,
                                                      int max_rectangle_in_subproblem) {

        std::vector<RectanglePlacement> current_solution = problem.get_current_solution();

        // 0. Base Case: If no reruns allowed, we return whatever the problem has (likely greedy result)
        if (num_reruns < 0) return current_solution;

        // Determine problem size
        std::unordered_set<int> unique_boxes;
        for (const auto& p : current_solution) unique_boxes.insert(p.box_id);

        int num_boxes = unique_boxes.size();
        int num_rectangles = current_solution.size();

        // Stop criteria for splitting: few boxes OR few rectangles
        bool is_small_subproblem = (num_boxes <= 2) || (num_rectangles < max_rectangle_in_subproblem);

        std::vector<RectanglePlacement> intermediate_result;

        // --- PHASE 1: Generate Solution (Local Search vs Divide & Conquer) ---

        if (is_small_subproblem) {
            // LEAF NODE: Apply Local Search
            intermediate_result = apply_local_search(problem, current_solution);
        }
        else {
            // INTERNAL NODE: Divide and Conquer
            auto [left, right] = split_rectangles_by_box_id(current_solution);
            int L = problem.get_box_length();

            // Create sub-problems
            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            // Recurse (Note: we pass num_reruns down, but the real cost happens in Phase 2)
            auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            // Merge
            intermediate_result = safe_merge(solved_left, solved_right);
        }

        // --- PHASE 2: The Optimization Rerun ---
        // If we have reruns left, we check for inefficiency and redo the bad parts.

        if (num_reruns > 0) {
            return filter_and_rerun(intermediate_result,
                                    problem.get_box_length(),
                                    num_reruns - 1,
                                    max_rectangle_in_subproblem);
        }

        return intermediate_result;
    }

    /**
     * Single step optimization (useful for visualization/GUI stepping).
     */
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T = 1000) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();
        auto neighbors = construct_neighbors(problem);

        if (neighbors.empty()) {
            // std::cout << "No neighbors generated." << std::endl;
            return solution;
        }

        // T < 1000 implies Simulated Annealing context, otherwise greedy
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
    // Pure virtual: Subclasses must define how to find neighbor solutions
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) = 0;

    // Threshold for "Good" boxes. Boxes filled > this % are kept as-is.
    virtual double get_utilization_threshold() const {
        return 0.75;
    }

private:
    // --- Phase 1 Helpers ---

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

        // If we found a better neighbor, update the problem state
        if (best_obj > current_obj) {
            problem.set_current_solution(best_sol);
            return best_sol;
        }
        return initial;
    }

    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    split_rectangles_by_box_id(const std::vector<RectanglePlacement>& solution) {
        std::vector<RectanglePlacement> left, right;
        if (solution.empty()) return {left, right};

        std::unordered_set<int> box_ids;
        for (const auto& p : solution) box_ids.insert(p.box_id);

        std::vector<int> sorted_ids(box_ids.begin(), box_ids.end());
        std::sort(sorted_ids.begin(), sorted_ids.end());

        // Split unique boxes in half
        int mid = sorted_ids.size() / 2;
        std::unordered_set<int> left_set;
        for(int i = 0; i < mid; ++i) left_set.insert(sorted_ids[i]);

        for (const auto& p : solution) {
            if (left_set.count(p.box_id)) left.push_back(p);
            else right.push_back(p);
        }
        return {left, right};
    }

    // --- Phase 2 Helpers (The Fix) ---

    /**
     * Identifies poorly utilized boxes, strips their identities, and reruns the solver
     * on just those rectangles. Combines the result with the "Good" boxes.
     */
    std::vector<RectanglePlacement> filter_and_rerun(const std::vector<RectanglePlacement>& full_solution,
                                                     int L,
                                                     int remaining_reruns,
                                                     int max_rect_sub) {

        // 1. Calculate Area per Box
        std::unordered_map<int, long long> box_areas;
        for (const auto& p : full_solution) {
            box_areas[p.box_id] += (long long)p.width * p.height;
        }

        long long box_capacity = (long long)L * L;
        double threshold = get_utilization_threshold();

        std::vector<RectanglePlacement> kept_good_rects;
        std::vector<RectanglePlacement> retry_bad_rects;

        // 2. Filter: High Util -> Keep, Low Util -> Retry
        for (const auto& p : full_solution) {
            double util = (double)box_areas[p.box_id] / (double)box_capacity;

            if (util >= threshold) {
                kept_good_rects.push_back(p);
            } else {
                retry_bad_rects.push_back(p);
            }
        }

        // Optimization: If nothing to retry, or everything is bad (avoid infinite loops if no progress), return.
        if (retry_bad_rects.empty()) {
            return full_solution;
        }

        // If kept_good_rects is empty, it means the whole solution is below threshold.
        // We still proceed to rerun, relying on remaining_reruns decrement to stop eventual loops.

        std::cout << "Rerun Optimization: Locking " << kept_good_rects.size()
                  << " rects, Rerunning " << retry_bad_rects.size() << " rects." << std::endl;

        // 3. RERUN: Create a new problem from the "Bad" rectangles.
        // This implicitly "merges" the bad boxes because the new solver doesn't care about old IDs.
        RectangleFittingProblem retry_problem(L, retry_bad_rects);

        // Recursive call with decremented reruns
        auto optimized_bad_part = solve_with_reruns(retry_problem, remaining_reruns, max_rect_sub);

        // 4. Merge the kept good part with the newly optimized part
        return safe_merge(kept_good_rects, optimized_bad_part);
    }

    /**
     * Merges two lists of placements, ensuring Box IDs in the 'right' list
     * do not conflict with those in the 'left' list.
     */
    std::vector<RectanglePlacement> safe_merge(const std::vector<RectanglePlacement>& left,
                                               const std::vector<RectanglePlacement>& right) {
        if (left.empty()) return right;
        if (right.empty()) return left;

        // Find max ID in left
        int max_left_id = -1;
        for (const auto& p : left) max_left_id = std::max(max_left_id, p.box_id);

        // Remap right to start after left
        auto remapped_right = remap_box_ids(right, max_left_id + 1);

        std::vector<RectanglePlacement> merged = left;
        merged.insert(merged.end(), remapped_right.begin(), remapped_right.end());
        return merged;
    }

    std::vector<RectanglePlacement> remap_box_ids(const std::vector<RectanglePlacement>& solution,
                                                  int start_id) {
        std::vector<RectanglePlacement> result;
        result.reserve(solution.size());

        std::unordered_map<int, int> id_map;
        int next_id = start_id;

        for (const auto& p : solution) {
            if (id_map.find(p.box_id) == id_map.end()) {
                id_map[p.box_id] = next_id++;
            }
            // Construct new placement with updated box_id
            result.emplace_back(p.width, p.height, p.x, p.y, p.rotated, id_map[p.box_id]);
        }
        return result;
    }
};

#endif //OPTIMALGO_SOLVER_H