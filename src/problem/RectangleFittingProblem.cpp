#include "RectangleFittingProblem.h"
#include <algorithm>
#include <climits>
#include <cmath>

double RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution) {
    return objective(solution, 1000);
}

double RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution, int T) {
    const double BOX_PENALTY = 10000;
    const double EDGE_BONUS = 2.0;

    if (solution.empty()) return 0;

    double box_capacity = (double)(L * L);
    std::unordered_map<int, int> box_area_used;
    int total_edge_touching = 0;
    int total_overlap_area = 0;

    for (const auto& rect : solution) {
        box_area_used[rect.box_id] += rect.width * rect.height;

        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x == 0) total_edge_touching += h;
        if (rect.y == 0) total_edge_touching += w;
        if (rect.x + w == L) total_edge_touching += h;
        if (rect.y + h == L) total_edge_touching += w;
    }

    double utilization_score = 0.0;
    for (const auto& [box_id, area] : box_area_used) {
        double util = std::min(1.0, area / box_capacity);
        utilization_score += util * util * 100.0;
    }

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
            } else if (x_overlap == 0 && y_overlap > 0) {
                total_edge_touching += y_overlap;
            } else if (y_overlap == 0 && x_overlap > 0) {
                total_edge_touching += x_overlap;
            }
        }
    }

    double temperature_factor = 1.0 - (T / 1000.0); // 0 when T=1000, 1 when T=0
    temperature_factor = std::max(0.0, std::min(1.0, temperature_factor));

    // Start punishing overlaps more heavily as temperature decreases
    double overlap_penalty_weight = temperature_factor * temperature_factor * 100000.0; // Quadratic growth
    double overlap_penalty = total_overlap_area * overlap_penalty_weight;

    double score = utilization_score * 20000 +
                   total_edge_touching * EDGE_BONUS -
                   box_area_used.size() * BOX_PENALTY -
                   overlap_penalty;

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
            return false;
        }
    }
    return true;
}

bool RectangleFittingProblem::solution_legal(const std::vector<RectanglePlacement>& current_solution) const {
    return check_no_overlaps(current_solution) && check_within_boxes(current_solution);
}
