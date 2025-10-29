#include "RectangleFittingProblem.h"
#include <algorithm>
#include <map>
#include <iostream>

bool RectangleFittingProblem::edges_touching(const RectanglePlacement r1, const RectanglePlacement r2) {
    // Get the actual dimensions considering rotation
    int r1_width = r1.get_actual_width();
    int r1_height = r1.get_actual_height();
    int r2_width = r2.get_actual_width();
    int r2_height = r2.get_actual_height();

    // Calculate the boundaries of each rectangle
    int r1_left = r1.x;
    int r1_right = r1.x + r1_width;
    int r1_top = r1.y;
    int r1_bottom = r1.y + r1_height;

    int r2_left = r2.x;
    int r2_right = r2.x + r2_width;
    int r2_top = r2.y;
    int r2_bottom = r2.y + r2_height;

    // Check if rectangles are adjacent (touching) but not overlapping

    // Case 1: Vertical alignment - same x-range and touching vertically
    if (r1_left == r2_left && r1_right == r2_right) {
        // r1 above r2, touching
        if (r1_bottom == r2_top) return true;
        // r1 below r2, touching
        if (r1_top == r2_bottom) return true;
    }

    // Case 2: Horizontal alignment - same y-range and touching horizontally
    if (r1_top == r2_top && r1_bottom == r2_bottom) {
        // r1 left of r2, touching
        if (r1_right == r2_left) return true;
        // r1 right of r2, touching
        if (r1_left == r2_right) return true;
    }
    return false;
}


int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& current_solution)  {
    int punishment_for_each_box = -10;
    int reward_for_touching_rectangles = 4;
    int reward_for_filling_boxes = 5;
    int objective = 0;

    std::vector<int> unique_boxes;
    for (const RectanglePlacement& rectangle_placement : current_solution) {
        unique_boxes.push_back(rectangle_placement.box_id);
    }
    std::sort(unique_boxes.begin(), unique_boxes.end());
    auto last = std::unique(unique_boxes.begin(), unique_boxes.end());
    unique_boxes.erase(last, unique_boxes.end());
    objective += punishment_for_each_box * unique_boxes.size();

    std::vector<int> coverages = calculate_cover_area_in_bounding_box(current_solution);
    for (int coverage : coverages) {
        objective += coverage / 10;
    }

    int n = current_solution.size();
    for (int i = 0; i < n; ++i) {
        const RectanglePlacement& rect1 = current_solution[i];
        if (rect1.x == 0 || rect1.x + rect1.get_actual_width() == L) {
            objective += reward_for_touching_rectangles;
        }
        if (rect1.y == 0 || rect1.y + rect1.get_actual_height() == L) {
            objective += reward_for_touching_rectangles;
        }
        for (int j = i + 1; j < n; ++j) {
            if (edges_touching(rect1, current_solution[j])) {
                objective += reward_for_touching_rectangles;
            }
        }
    }

    return objective;
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

std::vector<int> RectangleFittingProblem::calculate_cover_area_in_bounding_box(std::vector<RectanglePlacement> current_solution) {
    std::map<int, int> box_covered_area;

    for (const auto& rect : current_solution) {
        int box_id = rect.box_id;
        int rect_area = rect.get_actual_width() * rect.get_actual_height();
        box_covered_area[box_id] += rect_area;
    }

    std::vector<int> coverage_percentages;
    for (const auto& [box_id, covered_area] : box_covered_area) {
        int coverage_percentage = (covered_area * 100) / (L * L);
        coverage_percentages.push_back(coverage_percentage);
    }

    return coverage_percentages;
}
