#include "RectangleFittingProblem.h"
#include <algorithm>
#include <complex>
#include <map>
#include <iostream>


int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution) {
    auto boxes = group_rectangles_by_bounding_box(solution);
    int num_boxes = boxes.size();

    int score = -1000 * num_boxes;  // Heavy penalty for each box

    std::unordered_map<int, int> coverages =  get_coverage_each_bounding_box();


    for (const auto& coverage : get_coverage_each_bounding_box()) {
        if (coverage.second != 0) {
            //int bonus = (coverage.second / (L * L)) * (coverage.second / (L * L)) ; //Reward higher coverage more
            score += coverage.second * coverage.second;
            std::cout << "Coverage bonus " << coverage.second << std::endl;

        }

    }




    return score;
}

bool RectangleFittingProblem::check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const {
    for (size_t i = 0; i < current_solution.size(); ++i) {
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& a = current_solution[i];
            const auto& b = current_solution[j];
            if (a.box_id == b.box_id &&
                a.x < b.x + b.get_actual_width() &&
                a.x + a.get_actual_width() > b.x &&
                a.y < b.y + b.get_actual_height() &&
                a.y + a.get_actual_height() > b.y) {
                return false;
                }
        }
    }
    return true;
}

bool RectangleFittingProblem::check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const {
    for (const auto& rect : current_solution) {
        double right = rect.x + rect.get_actual_width();
        double bottom = rect.y + rect.get_actual_height();

        if (rect.x < 0 || rect.y < 0 || right > L || bottom > L) {
            std::cout << "Rectangle out of bounds: (" << rect.x << ", " << rect.y
                      << ") to (" << right << ", " << bottom << ") in box [0,0] to ["
                      << L << "," << L << "]\n";
            return false;
        }
    }
    return true;
}

bool RectangleFittingProblem::solution_legal(const std::vector<RectanglePlacement>& current_solution)
    const{ return check_no_overlaps(current_solution) && check_within_boxes(current_solution); };

