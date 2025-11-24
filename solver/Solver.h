//
// Solver.h
// Base class for rectangle packing solvers with common divide-and-conquer logic
//

#ifndef OPTIMALGO_SOLVER_H
#define OPTIMALGO_SOLVER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>

class Solver {
public:
    virtual ~Solver() = default;

    // Main entry point - delegates to solve_with_reruns
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem,
                                          int num_reruns,
                                          int max_rectangle_in_subproblem) {
        return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
    }

    // Recursive divide-and-conquer solver with optimization passes
    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem,
                                                      int num_reruns,
                                                      int max_rectangle_in_subproblem) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();

        if (num_reruns <= 0) return solution;

        std::unordered_set<int> box_ids;
        for (const auto& placement : solution) {
            box_ids.insert(placement.box_id);
        }

        int num_boxes = box_ids.size();
        int num_rectangles = solution.size();

        // UPDATED: Stop criteria - either few boxes OR few rectangles
        // This prevents premature termination when you have many poorly-utilized boxes
        bool small_enough = (num_boxes <= 2) || (num_rectangles < max_rectangle_in_subproblem);

        // A. Subproblem size is small enough: Apply local search
        if (small_enough) {
            auto neighbors = construct_neighbors(problem);
            if (neighbors.empty()) {
                std::cout << "No neighbors generated, stopping." << std::endl;
                return solution;
            }

            int current_obj = problem.objective(solution);
            std::vector<RectanglePlacement> best_neighbor = solution;
            int best_obj = current_obj;

            for (auto &n : neighbors) {
                int obj = problem.objective(n);
                if (obj > best_obj) {
                    best_obj = obj;
                    best_neighbor = n;
                }
            }

            if (best_obj > current_obj) {
                // Continue local search with the new, better solution
                problem.set_current_solution(best_neighbor);
                return solve_with_reruns(problem, num_reruns, max_rectangle_in_subproblem);
            } else {
                // Local optimum reached
                return solution;
            }
        }
        // B. Subproblem is too large: Divide-and-Conquer
        else {
            // --- Divide ---
            auto [left, right] = splitRectanglesByBoxId(solution);
            int L = problem.get_box_length();

            RectangleFittingProblem p1(L, left);
            RectangleFittingProblem p2(L, right);

            // --- Conquer (Recursive Calls) ---
            auto solved_left = solve_with_reruns(p1, num_reruns, max_rectangle_in_subproblem);
            auto solved_right = solve_with_reruns(p2, num_reruns, max_rectangle_in_subproblem);

            // --- Merge ---
            // REMAP BOX IDs to avoid conflicts when merging
            int max_left_id = 0;
            for (const auto& placement : solved_left) {
                max_left_id = std::max(max_left_id, placement.box_id);
            }
            auto remapped_right = remap_box_ids(solved_right, max_left_id + 1);

            std::vector<RectanglePlacement> merged = solved_left;
            merged.insert(merged.end(), remapped_right.begin(), remapped_right.end());

            // --- Optimize Merged Solution ---
            if (num_reruns > 0) {
                return optimize_merged_solution(merged, L, num_reruns, max_rectangle_in_subproblem);
            }

            problem.set_current_solution(merged);
            return merged;
        }
    }

    // Single step optimization (for iterative/GUI use)
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();

        auto neighbors = construct_neighbors(problem);
        if (neighbors.empty()) {
            std::cout << "No neighbors generated." << std::endl;
            return solution;
        }

        int current_obj = problem.objective(solution);
        std::vector<RectanglePlacement> best_neighbor = solution;
        int best_obj = current_obj;

        for (auto &n : neighbors) {
            int obj = problem.objective(n);
            if (obj > best_obj) {
                best_obj = obj;
                best_neighbor = n;
            }
        }

        problem.set_current_solution(best_neighbor);
        return best_neighbor;
    }

protected:
    // Pure virtual - each solver implements its own neighborhood structure
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem) = 0;

    // Utilization threshold for filtering well-utilized boxes
    // Subclasses can override this if needed
    virtual double get_utilization_threshold() const {
        return 0.75;
    }

    // Split rectangles by box ID into two balanced groups
    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    splitRectanglesByBoxId(const std::vector<RectanglePlacement>& solution) {
        std::vector<RectanglePlacement> left, right;

        if (solution.empty()) return {left, right};

        // Find all unique box IDs
        std::unordered_set<int> box_ids;
        for (const auto& placement : solution) {
            box_ids.insert(placement.box_id);
        }

        // Split boxes into two groups
        std::vector<int> sorted_box_ids(box_ids.begin(), box_ids.end());
        std::sort(sorted_box_ids.begin(), sorted_box_ids.end());

        int mid = sorted_box_ids.size() / 2;

        std::unordered_set<int> left_box_ids, right_box_ids;
        for (int i = 0; i < mid; i++) {
            left_box_ids.insert(sorted_box_ids[i]);
        }
        for (size_t i = mid; i < sorted_box_ids.size(); i++) {
            right_box_ids.insert(sorted_box_ids[i]);
        }

        // Assign rectangles to left or right based on their box ID
        for (const auto& placement : solution) {
            if (left_box_ids.count(placement.box_id)) {
                left.push_back(placement);
            } else {
                right.push_back(placement);
            }
        }

        return {left, right};
    }

    // Remap box IDs to avoid conflicts when merging solutions
    std::vector<RectanglePlacement> remap_box_ids(
        const std::vector<RectanglePlacement>& solution,
        int start_id) {

        std::vector<RectanglePlacement> result;
        std::unordered_map<int, int> id_mapping;
        int next_id = start_id;

        for (const auto& placement : solution) {
            if (id_mapping.find(placement.box_id) == id_mapping.end()) {
                id_mapping[placement.box_id] = next_id++;
            }
            int new_box_id = id_mapping[placement.box_id];
            result.emplace_back(placement.width, placement.height, placement.x, placement.y,
                               placement.rotated, new_box_id);
        }

        return result;
    }

private:
    // Group poorly-utilized boxes together with constraints on both utilization and rectangle count
    std::vector<std::vector<int>> group_boxes_by_combined_utilization(
        const std::unordered_map<int, long long>& box_areas,
        const std::unordered_map<int, int>& box_rect_counts,
        long long box_capacity,
        int max_rectangles_per_group,
        double combined_threshold = 0.80) {

        std::vector<std::vector<int>> groups;

        // Sort boxes by area (smallest first for better grouping)
        std::vector<std::pair<int, long long>> sorted_boxes;
        for (const auto& [box_id, area] : box_areas) {
            double utilization = (double)area / (double)box_capacity;
            // Only group boxes with low utilization
            if (utilization < get_utilization_threshold()) {
                sorted_boxes.push_back({box_id, area});
            }
        }

        std::sort(sorted_boxes.begin(), sorted_boxes.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });

        std::vector<bool> used(sorted_boxes.size(), false);

        // Greedy grouping: try to combine boxes up to BOTH thresholds
        for (size_t i = 0; i < sorted_boxes.size(); i++) {
            if (used[i]) continue;

            std::vector<int> current_group;
            long long combined_area = 0;
            int combined_rect_count = 0;

            current_group.push_back(sorted_boxes[i].first);
            combined_area += sorted_boxes[i].second;
            combined_rect_count += box_rect_counts.at(sorted_boxes[i].first);
            used[i] = true;

            // Try to add more boxes to this group
            for (size_t j = i + 1; j < sorted_boxes.size(); j++) {
                if (used[j]) continue;

                int box_id_j = sorted_boxes[j].first;
                long long potential_area = combined_area + sorted_boxes[j].second;
                int potential_rect_count = combined_rect_count + box_rect_counts.at(box_id_j);
                double potential_utilization = (double)potential_area / (double)box_capacity;

                // CHECK BOTH: utilization threshold AND rectangle count limit
                if (potential_utilization <= combined_threshold &&
                    potential_rect_count <= max_rectangles_per_group) {
                    current_group.push_back(box_id_j);
                    combined_area += sorted_boxes[j].second;
                    combined_rect_count += box_rect_counts.at(box_id_j);
                    used[j] = true;
                }
            }

            // Only add group if it has 2+ boxes
            if (current_group.size() >= 2) {
                groups.push_back(current_group);
            }
        }

        return groups;
    }

    // Optimize merged solution by grouping poorly-utilized boxes and re-optimizing them
    std::vector<RectanglePlacement> optimize_merged_solution(
        const std::vector<RectanglePlacement>& merged,
        int L,
        int num_reruns,
        int max_rectangle_in_subproblem) {

        // Create a new problem with the merged solution
        RectangleFittingProblem merged_problem(L, merged);

        // Calculate coverage for each box
        auto coverage = merged_problem.get_coverage_each_bounding_box();

        // Group boxes by their ID and calculate total areas AND rectangle counts
        std::unordered_map<int, long long> box_areas;
        std::unordered_map<int, int> box_rect_counts;
        for (const auto& placement : merged) {
            box_areas[placement.box_id] = coverage[placement.box_id];
            box_rect_counts[placement.box_id]++;
        }

        long long box_capacity = (long long)L * L;
        double utilization_threshold = get_utilization_threshold();

        // Separate well-utilized and poorly-utilized rectangles
        std::vector<RectanglePlacement> well_utilized_rectangles;
        std::unordered_set<int> poorly_utilized_box_ids;

        for (const auto& [box_id, area] : box_areas) {
            double util = (double)area / (double)box_capacity;
            if (util < utilization_threshold) {
                poorly_utilized_box_ids.insert(box_id);
            }
        }

        std::vector<RectanglePlacement> poorly_utilized_rectangles;
        for (const auto& placement : merged) {
            if (poorly_utilized_box_ids.count(placement.box_id)) {
                poorly_utilized_rectangles.push_back(placement);
            } else {
                well_utilized_rectangles.push_back(placement);
            }
        }

        std::cout << "After merging: " << well_utilized_rectangles.size()
                  << " rectangles in well-utilized boxes (>=" << (utilization_threshold * 100) << "%), "
                  << poorly_utilized_rectangles.size()
                  << " rectangles in " << poorly_utilized_box_ids.size()
                  << " poorly-utilized boxes" << std::endl;

        // Group and optimize poorly-utilized boxes
        if (!poorly_utilized_rectangles.empty() && num_reruns > 0) {
            // Try to group poorly-utilized boxes with BOTH utilization and rectangle constraints
            auto box_groups = group_boxes_by_combined_utilization(
                box_areas,
                box_rect_counts,
                box_capacity,
                max_rectangle_in_subproblem,  // Pass the rectangle limit!
                0.80
            );

            std::cout << "Created " << box_groups.size()
                      << " optimization groups from poorly-utilized boxes (max "
                      << max_rectangle_in_subproblem << " rectangles per group)" << std::endl;

            std::vector<RectanglePlacement> optimized_rectangles;
            std::unordered_set<int> optimized_box_ids;

            // Optimize each group
            for (size_t g = 0; g < box_groups.size(); g++) {
                const auto& group = box_groups[g];
                std::vector<RectanglePlacement> group_rectangles;
                long long group_total_area = 0;

                for (const auto& placement : poorly_utilized_rectangles) {
                    if (std::find(group.begin(), group.end(), placement.box_id) != group.end()) {
                        group_rectangles.push_back(placement);
                        optimized_box_ids.insert(placement.box_id);
                    }
                }

                for (int box_id : group) {
                    group_total_area += box_areas[box_id];
                }

                if (!group_rectangles.empty()) {
                    double group_util = (double)group_total_area / (double)box_capacity;
                    std::cout << "  Group " << (g+1) << ": optimizing " << group_rectangles.size()
                              << " rectangles from " << group.size()
                              << " boxes (combined util: " << (group_util * 100) << "%, "
                              << "rectangles: " << group_rectangles.size() << ")" << std::endl;

                    RectangleFittingProblem group_problem(L, group_rectangles);
                    auto optimized_group = solve_with_reruns(group_problem, num_reruns - 1, max_rectangle_in_subproblem);
                    optimized_rectangles.insert(optimized_rectangles.end(),
                                               optimized_group.begin(),
                                               optimized_group.end());
                }
            }

            // Add rectangles from boxes that weren't grouped (too large or alone)
            for (const auto& placement : poorly_utilized_rectangles) {
                if (!optimized_box_ids.count(placement.box_id)) {
                    optimized_rectangles.push_back(placement);
                }
            }

            // Combine well-utilized boxes with optimized rectangles
            std::vector<RectanglePlacement> final_solution = well_utilized_rectangles;

            // Remap box IDs for the optimized part to avoid conflicts
            int final_max_id = 0;
            for (const auto& placement : final_solution) {
                final_max_id = std::max(final_max_id, placement.box_id);
            }
            auto remapped_optimized = remap_box_ids(optimized_rectangles, final_max_id + 1);

            final_solution.insert(final_solution.end(), remapped_optimized.begin(), remapped_optimized.end());

            return final_solution;
        }

        // If all boxes are well utilized or no reruns left, return merged solution
        return merged;
    }
};

#endif //OPTIMALGO_SOLVER_H