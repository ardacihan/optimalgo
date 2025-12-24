#include "RectangleFittingProblem.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <set>
int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution) {
    return objective(solution, 1000);
}

int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution, int T) {
    const int BOX_PENALTY = 10000000;
    const double TOUCHING_BONUS = 2.0;
    const double SURFACE_BONUS = 1.5;

    if (solution.empty()) return 0;

    long long box_capacity = (long long)L * L;

    // Calculate boxes used and area per box
    std::set<int> boxes_used;
    std::unordered_map<int, int> box_area_used;

    for (const auto& rect : solution) {
        boxes_used.insert(rect.box_id);
        int area = rect.width * rect.height;
        box_area_used[rect.box_id] += area;
    }

    // Calculate utilization score
    double utilization_score = 0.0;
    for (int box_id : boxes_used) {
        double util = (double)box_area_used[box_id] / box_capacity;
        if (util > 1.0) util = 0.0;
        utilization_score += std::pow(util, 3);
    }

    // Calculate touching lengths and surface touching
    int total_touching_length = 0;
    int total_surface_touching = 0;

    for (size_t i = 0; i < solution.size(); i++) {
        const auto& r1 = solution[i];
        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        // Check edges against box boundaries
        if (r1.x == 0) total_surface_touching += h1;
        if (r1.y == 0) total_surface_touching += w1;
        if (r1.x + w1 == L) total_surface_touching += h1;
        if (r1.y + h1 == L) total_surface_touching += w1;

        // Check against other rectangles in same box
        for (size_t j = i + 1; j < solution.size(); j++) {
            const auto& r2 = solution[j];
            if (r1.box_id != r2.box_id) continue;

            int w2 = r2.get_actual_width();
            int h2 = r2.get_actual_height();

            // Check horizontal touching (shared vertical edge)
            if (r1.x + w1 == r2.x || r2.x + w2 == r1.x) {
                int y_overlap = std::min(r1.y + h1, r2.y + h2) - std::max(r1.y, r2.y);
                if (y_overlap > 0) {
                    total_touching_length += y_overlap;
                }
            }

            // Check vertical touching (shared horizontal edge)
            if (r1.y + h1 == r2.y || r2.y + h2 == r1.y) {
                int x_overlap = std::min(r1.x + w1, r2.x + w2) - std::max(r1.x, r2.x);
                if (x_overlap > 0) {
                    total_touching_length += x_overlap;
                }
            }
        }
    }

    // Calculate overlap area
    int total_overlap_area = 0;
    for (size_t i = 0; i < solution.size(); i++) {
        const auto& r1 = solution[i];
        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        for (size_t j = i + 1; j < solution.size(); j++) {
            const auto& r2 = solution[j];
            if (r1.box_id != r2.box_id) continue;

            int w2 = r2.get_actual_width();
            int h2 = r2.get_actual_height();

            int x_overlap = std::min(r1.x + w1, r2.x + w2) - std::max(r1.x, r2.x);
            int y_overlap = std::min(r1.y + h1, r2.y + h2) - std::max(r1.y, r2.y);

            if (x_overlap > 0 && y_overlap > 0) {
                total_overlap_area += x_overlap * y_overlap;
            }
        }
    }

    int num_boxes = boxes_used.size();

    double overlap_penalty = total_overlap_area * std::exp(2000.0 / std::max(T, 1)) * 1000;
    double score = utilization_score * 20000 +
                   total_touching_length * TOUCHING_BONUS +
                   total_surface_touching * SURFACE_BONUS -
                   num_boxes * BOX_PENALTY -
                   overlap_penalty;

    return (int)std::clamp(score, (double)INT_MIN/2.0, (double)INT_MAX/2.0);
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
            return false;
        }
    }
    return true;
}

bool RectangleFittingProblem::solution_legal(const std::vector<RectanglePlacement>& current_solution) const {
    return check_no_overlaps(current_solution) && check_within_boxes(current_solution);
}
