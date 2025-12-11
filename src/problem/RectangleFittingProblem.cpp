#include "RectangleFittingProblem.h"
#include <algorithm>
#include <climits>
#include <cmath>

void RectangleFittingProblem::compute_full_metrics(
    const std::vector<RectanglePlacement>& solution, int T) {

    cached_metrics = SolutionMetrics();
    cached_metrics.last_computed_T = T;

    if (solution.empty()) {
        metrics_valid = true;
        return;
    }

    for (size_t i = 0; i < solution.size(); ++i) {
        const auto& r1 = solution[i];
        int box_id = r1.box_id;
        cached_metrics.boxes_used.insert(box_id);
        cached_metrics.box_area_used[box_id] += (long long)r1.width * r1.height;

        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        // Surface touching bonus
        if (r1.x == 0) cached_metrics.total_surface_touching += h1;
        if (r1.x + w1 == L) cached_metrics.total_surface_touching += h1;
        if (r1.y == 0) cached_metrics.total_surface_touching += w1;
        if (r1.y + h1 == L) cached_metrics.total_surface_touching += w1;

        for (size_t j = i + 1; j < solution.size(); ++j) {
            const auto& r2 = solution[j];
            if (box_id != r2.box_id) continue;

            // Overlap
            int ox1 = std::max(r1.x, r2.x);
            int oy1 = std::max(r1.y, r2.y);
            int ox2 = std::min(r1.x + w1, r2.x + r2.get_actual_width());
            int oy2 = std::min(r1.y + h1, r2.y + r2.get_actual_height());
            if (ox1 < ox2 && oy1 < oy2) {
                cached_metrics.total_overlap_area += (long long)(ox2 - ox1) * (oy2 - oy1);
            }

            // Rectangle-to-rectangle touching
            if (r1.adjacent(r2)) {
                if (r1.x + w1 == r2.x || r2.x + r2.get_actual_width() == r1.x)
                    cached_metrics.total_touching_length += std::min(h1, r2.get_actual_height());
                if (r1.y + h1 == r2.y || r2.y + r2.get_actual_height() == r1.y)
                    cached_metrics.total_touching_length += std::min(w1, r2.get_actual_width());
            }
        }
    }

    metrics_valid = true;
}

int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution) {
    return objective(solution, 1000);
}

int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& solution, int T) {
    const int BOX_PENALTY = 1000000;
    const double TOUCHING_BONUS = 8.0;
    const double SURFACE_BONUS = 5.0;

    if (solution.empty()) return 0;
    if (!metrics_valid || cached_metrics.last_computed_T != T)
        compute_full_metrics(solution, T);

    int num_boxes = cached_metrics.boxes_used.size();
    long long box_capacity = (long long)L * L;

    double utilization_score = 0.0;
    for (int box_id : cached_metrics.boxes_used) {
        double util = (double)cached_metrics.box_area_used[box_id] / box_capacity;
        if (util > 1.0) util = 0.0;
        utilization_score += std::pow(util, 3); // exponential reward
    }

    // Overlap penalty grows exponentially with shrinking T
    double overlap_penalty = cached_metrics.total_overlap_area * std::exp(2000.0 / std::max(T, 1)) * 1000;
    double score = utilization_score * 20000 +
                   cached_metrics.total_touching_length * TOUCHING_BONUS +
                   cached_metrics.total_surface_touching * SURFACE_BONUS -
                   num_boxes * BOX_PENALTY -
                   overlap_penalty;

    return (int)std::clamp(score, (double)INT_MIN/2.0, (double)INT_MAX/2.0);
}

int RectangleFittingProblem::objective_delta(
    const std::vector<RectanglePlacement>& new_solution,
    int changed_rect_idx, int T) {

    if (!metrics_valid || cached_metrics.last_computed_T != T)
        return objective(new_solution, T);

    const auto& old_rect = current_solution[changed_rect_idx];
    const auto& new_rect = new_solution[changed_rect_idx];

    if (old_rect.x == new_rect.x && old_rect.y == new_rect.y &&
        old_rect.rotated == new_rect.rotated && old_rect.box_id == new_rect.box_id) {
        return objective(current_solution, T);
    }

    SolutionMetrics delta_metrics = cached_metrics;

    int old_box = old_rect.box_id;
    int new_box = new_rect.box_id;

    int old_w = old_rect.get_actual_width();
    int old_h = old_rect.get_actual_height();
    int new_w = new_rect.get_actual_width();
    int new_h = new_rect.get_actual_height();

    // Remove old rectangle
    if (old_rect.x == 0) delta_metrics.total_surface_touching -= old_h;
    if (old_rect.x + old_w == L) delta_metrics.total_surface_touching -= old_h;
    if (old_rect.y == 0) delta_metrics.total_surface_touching -= old_w;
    if (old_rect.y + old_h == L) delta_metrics.total_surface_touching -= old_w;

    for (size_t j = 0; j < current_solution.size(); ++j) {
        if (j == changed_rect_idx) continue;
        const auto& r2 = current_solution[j];
        if (r2.box_id != old_box) continue;

        // Overlap
        int ox1 = std::max(old_rect.x, r2.x);
        int oy1 = std::max(old_rect.y, r2.y);
        int ox2 = std::min(old_rect.x + old_w, r2.x + r2.get_actual_width());
        int oy2 = std::min(old_rect.y + old_h, r2.y + r2.get_actual_height());
        if (ox1 < ox2 && oy1 < oy2)
            delta_metrics.total_overlap_area -= (long long)(ox2 - ox1) * (oy2 - oy1);

        if (old_rect.adjacent(r2)) {
            if (old_rect.x + old_w == r2.x || r2.x + r2.get_actual_width() == old_rect.x)
                delta_metrics.total_touching_length -= std::min(old_h, r2.get_actual_height());
            if (old_rect.y + old_h == r2.y || r2.y + r2.get_actual_height() == old_rect.y)
                delta_metrics.total_touching_length -= std::min(old_w, r2.get_actual_width());
        }
    }

    // Add new rectangle
    if (new_rect.x == 0) delta_metrics.total_surface_touching += new_h;
    if (new_rect.x + new_w == L) delta_metrics.total_surface_touching += new_h;
    if (new_rect.y == 0) delta_metrics.total_surface_touching += new_w;
    if (new_rect.y + new_h == L) delta_metrics.total_surface_touching += new_w;

    for (size_t j = 0; j < new_solution.size(); ++j) {
        if (j == changed_rect_idx) continue;
        const auto& r2 = new_solution[j];
        if (r2.box_id != new_box) continue;

        int ox1 = std::max(new_rect.x, r2.x);
        int oy1 = std::max(new_rect.y, r2.y);
        int ox2 = std::min(new_rect.x + new_w, r2.x + r2.get_actual_width());
        int oy2 = std::min(new_rect.y + new_h, r2.y + r2.get_actual_height());
        if (ox1 < ox2 && oy1 < oy2)
            delta_metrics.total_overlap_area += (long long)(ox2 - ox1) * (oy2 - oy1);

        if (new_rect.adjacent(r2)) {
            if (new_rect.x + new_w == r2.x || r2.x + r2.get_actual_width() == new_rect.x)
                delta_metrics.total_touching_length += std::min(new_h, r2.get_actual_height());
            if (new_rect.y + new_h == r2.y || r2.y + r2.get_actual_height() == new_rect.y)
                delta_metrics.total_touching_length += std::min(new_w, r2.get_actual_width());
        }
    }

    // Update box area
    if (old_box != new_box) {
        delta_metrics.box_area_used[old_box] -= (long long)old_rect.width * old_rect.height;
        if (delta_metrics.box_area_used[old_box] == 0) {
            delta_metrics.box_area_used.erase(old_box);
            delta_metrics.boxes_used.erase(old_box);
        }

        delta_metrics.box_area_used[new_box] += (long long)new_rect.width * new_rect.height;
        delta_metrics.boxes_used.insert(new_box);
    }

    int num_boxes = delta_metrics.boxes_used.size();
    long long box_capacity = (long long)L * L;

    double utilization_score = 0.0;
    for (int box_id : delta_metrics.boxes_used) {
        double util = (double)delta_metrics.box_area_used[box_id] / box_capacity;
        if (util > 1.0) util = 0.0;
        utilization_score += std::pow(util, 3);
    }

    double overlap_penalty = delta_metrics.total_overlap_area * std::exp(2000.0 / std::max(T, 1));

    double score = utilization_score * 14000 +
                   delta_metrics.total_touching_length * 5 +
                   delta_metrics.total_surface_touching * 3 -
                   num_boxes * 1000000 -
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
