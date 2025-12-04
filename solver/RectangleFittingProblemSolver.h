//
// Created by arda on 11/24/25.
//


#ifndef OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H
#define OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H


#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "../problem/RectangleFittingProblem.h"


class RectangleFittingProblemSolver {
public:
    virtual ~RectangleFittingProblemSolver() {}

    virtual std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                                  int max_rectangle_in_subproblem, int T) {return std::vector<RectanglePlacement>();};

    virtual std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem, int T) {return std::vector<RectanglePlacement>();};

    double get_utilization_threshold() const {
        return 0.75;
    }


    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    split_rectangles_by_box_id(const std::vector<RectanglePlacement>& solution) {
        std::vector<RectanglePlacement> left, right;
        if (solution.empty()) return {left, right};

        std::unordered_set<int> box_ids;
        for (const auto& p : solution) box_ids.insert(p.box_id);

        std::vector<int> sorted_ids(box_ids.begin(), box_ids.end());
        std::sort(sorted_ids.begin(), sorted_ids.end());

        int mid = sorted_ids.size() / 2;
        std::unordered_set<int> left_set;
        for(int i = 0; i < mid; ++i) left_set.insert(sorted_ids[i]);

        for (const auto& p : solution) {
            if (left_set.count(p.box_id)) left.push_back(p);
            else right.push_back(p);
        }
        return {left, right};
    }


    std::vector<RectanglePlacement> filter_and_rerun(const std::vector<RectanglePlacement>& full_solution,
                                                     int L,
                                                     int remaining_reruns,
                                                     int max_rect_sub, int T) {

        // 1. Identify Broken Boxes (Overlaps)
        // Even if utilization is high, an overlap makes the box "Bad".
        std::unordered_set<int> boxes_with_overlaps;
        for (size_t i = 0; i < full_solution.size(); ++i) {
            for (size_t j = i + 1; j < full_solution.size(); ++j) {
                const auto& r1 = full_solution[i];
                const auto& r2 = full_solution[j];

                // Only check if in same box
                if (r1.box_id != r2.box_id) continue;

                // Simple intersection logic
                if (r1.x < r2.x + r2.get_actual_width() &&
                    r1.x + r1.get_actual_width() > r2.x &&
                    r1.y < r2.y + r2.get_actual_height() &&
                    r1.y + r1.get_actual_height() > r2.y) {

                    boxes_with_overlaps.insert(r1.box_id);
                }
            }
        }

        // 2. Calculate Area per Box
        std::unordered_map<int, long long> box_areas;
        for (const auto& p : full_solution) {
            box_areas[p.box_id] += (long long)p.width * p.height;
        }

        long long box_capacity = (long long)L * L;
        double threshold = get_utilization_threshold();

        std::vector<RectanglePlacement> kept_good_rects;
        std::vector<RectanglePlacement> retry_bad_rects;

        // 3. Filter: Keep ONLY if High Util AND No Overlaps
        for (const auto& p : full_solution) {
            double util = (double)box_areas[p.box_id] / (double)box_capacity;
            bool has_overlap = boxes_with_overlaps.count(p.box_id) > 0;

            // STRICT CONDITION: Utilization > Threshold AND Box is valid
            if (util >= threshold && !has_overlap) {
                kept_good_rects.push_back(p);
            } else {
                retry_bad_rects.push_back(p);
            }
        }

        if (retry_bad_rects.empty()) {
            return full_solution;
        }

        std::cout << "Rerun Optimization: Locking " << kept_good_rects.size()
                  << " rects, Rerunning " << retry_bad_rects.size()
                  << " rects (found " << boxes_with_overlaps.size() << " overlapping boxes)." << std::endl;

        // 4. RERUN on the bad/overlapping pile
        RectangleFittingProblem retry_problem(L, retry_bad_rects);

        // IMPORTANT: Recursively solve again.
        // This gives the solver a chance to fix the overlaps at a lower recursion depth.
        auto optimized_bad_part = solve_with_reruns(retry_problem, remaining_reruns, max_rect_sub,T);

        return safe_merge(kept_good_rects, optimized_bad_part);
    }

    std::vector<RectanglePlacement> safe_merge(const std::vector<RectanglePlacement>& left,
                                               const std::vector<RectanglePlacement>& right) {
        if (left.empty()) return right;
        if (right.empty()) return left;

        int max_left_id = -1;
        for (const auto& p : left) max_left_id = std::max(max_left_id, p.box_id);

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
            result.emplace_back(p.width, p.height, p.x, p.y, p.rotated, id_map[p.box_id]);
        }
        return result;
    }
};



#endif //OPTIMALGO_RECTANGLEFITTINGPROBLEMSOLVER_H