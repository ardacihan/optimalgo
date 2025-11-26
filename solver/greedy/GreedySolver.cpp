//
// Created by arda on 11/24/25.
//

#include "GreedySolver.h"


RectanglePlacement BiggestFirstSelectionStrategy::select_rectangle(RectangleFittingProblem &problem) {
    auto solution = problem.get_current_solution();

    std::unordered_map<int, std::vector<const RectanglePlacement*>> boxes;
    for (const auto& rect : solution) {
        boxes[rect.box_id].push_back(&rect);
    }

    std::vector<const RectanglePlacement*> candidate_rects;

    for (const auto& rect : solution) {
        auto box_it = boxes.find(rect.box_id);
        if (box_it != boxes.end() && box_it->second.size() == 1) {
            // This rectangle is the only one in its box
            candidate_rects.push_back(&rect);
        }
    }

    if (candidate_rects.empty()) {
        for (const auto& rect : solution) {
            candidate_rects.push_back(&rect);
        }
    }

    const RectanglePlacement* largest = candidate_rects[0];
    int max_area = largest->get_actual_width() * largest->get_actual_height();

    for (size_t i = 1; i < candidate_rects.size(); i++) {
        int area = candidate_rects[i]->get_actual_width() * candidate_rects[i]->get_actual_height();
        if (area > max_area) {
            max_area = area;
            largest = candidate_rects[i];
        }
    }

    return *largest;
}

double MaximizeContactSelectionStrategy::calculate_simple_score(const RectanglePlacement& rect, bool is_alone) {
    int area = rect.get_actual_width() * rect.get_actual_height();
    double aspect = (double)std::max(rect.width, rect.height) /
                   std::min(rect.width, rect.height);

    // Base score on area (larger is better)
    double score = std::sqrt(area);

    // Bonus for being alone in box (we want to fill these boxes)
    if (is_alone) score *= 1.5;

    // Penalty for extreme aspect ratios
    score /= (1.0 + (aspect - 1.0) * 0.1);

    return score;
}

RectanglePlacement MaximizeContactSelectionStrategy::select_rectangle(RectangleFittingProblem &problem) {
    auto solution = problem.get_current_solution();


    // Find rectangles that are alone in their boxes
    std::unordered_map<int, int> box_counts;
    for (const auto& rect : solution) {
        box_counts[rect.box_id]++;
    }

    const RectanglePlacement* best_fit = &solution[0];
    double best_score = -1;

    for (const auto& rect : solution) {
        // Check if rectangle is alone in its box
        bool is_alone = (box_counts[rect.box_id] == 1);

        double score = calculate_simple_score(rect, is_alone);
        if (score > best_score) {
            best_score = score;
            best_fit = &rect;
        }
    }

    return *best_fit;
}


void RightTopPlacementStrategy::place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle) {
    rectangle.x = 0;
    rectangle.y = 0;
    rectangle.box_id = 0;
}

std::vector<RectanglePlacement> GreedySolver::solve(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
    auto solution = problem.get_current_solution();
    return solution;
}

std::vector<RectanglePlacement> GreedySolver::solve_with_reruns(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem) {
    return solve(problem, num_reruns, max_rectangle_in_subproblem);
}