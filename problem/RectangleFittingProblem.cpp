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

    // 1. Calculate basic metrics
    for (size_t i = 0; i < solution.size(); ++i) {
        const auto& r1 = solution[i];
        int box_id = r1.box_id;
        cached_metrics.boxes_used.insert(box_id);
        cached_metrics.box_area_used[box_id] += (long long)r1.width * r1.height;

        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        // Surface touching
        if (r1.x == 0) cached_metrics.total_surface_touching += h1;
        if (r1.x + w1 == L) cached_metrics.total_surface_touching += h1;
        if (r1.y == 0) cached_metrics.total_surface_touching += w1;
        if (r1.y + h1 == L) cached_metrics.total_surface_touching += w1;

        // Check overlaps and adjacencies
        for (size_t j = i + 1; j < solution.size(); ++j) {
            const auto& r2 = solution[j];
            if (box_id != r2.box_id) continue;

            // Overlap calculation
            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + w1, r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + h1, r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                cached_metrics.total_overlap_area +=
                    (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }

            // Rectangle-to-rectangle touching
            if (r1.adjacent(r2)) {
                if (r1.x + w1 == r2.x || r2.x + r2.get_actual_width() == r1.x) {
                    cached_metrics.total_touching_length += std::min(h1, r2.get_actual_height());
                }
                if (r1.y + h1 == r2.y || r2.y + r2.get_actual_height() == r1.y) {
                    cached_metrics.total_touching_length += std::min(w1, r2.get_actual_width());
                }
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
    const int TOUCHING_BONUS = 5;
    const int SURFACE_BONUS = 3;

    if (solution.empty()) return 0;

    // Compute full metrics if not cached or temperature changed
    if (!metrics_valid || cached_metrics.last_computed_T != T) {
        compute_full_metrics(solution, T);
    }

    int num_boxes = cached_metrics.boxes_used.size();
    long long box_capacity = (long long)L * L;

    // Calculate utilization score
    long long utilization_score = 0;
    for (int box_id : cached_metrics.boxes_used) {
        double area_used = (double)cached_metrics.box_area_used[box_id];
        double utilization = area_used / box_capacity;

        if (utilization > 1.0) {
            utilization = 0.0;
        }

        utilization_score += (long long)(utilization * utilization * 10000);
    }

    // Temperature-dependent overlap penalty
    int overlap_penalty_per_unit = 10;
    if (T < 1800) overlap_penalty_per_unit = 20;
    if (T < 1600) overlap_penalty_per_unit = 50;
    if (T < 1400) overlap_penalty_per_unit = 100;
    if (T < 1200) overlap_penalty_per_unit = 200;
    if (T < 1000) overlap_penalty_per_unit = 500;
    if (T < 800) overlap_penalty_per_unit = 1000;
    if (T < 600) overlap_penalty_per_unit = 2000;
    if (T < 400) overlap_penalty_per_unit = 5000;
    if (T < 200) overlap_penalty_per_unit = 10000;
    if (T < 100) overlap_penalty_per_unit = 50000;
    if (T < 50) overlap_penalty_per_unit = 1000000;

    long long overlap_penalty = cached_metrics.total_overlap_area * overlap_penalty_per_unit;

    long long score =
        utilization_score +
        cached_metrics.total_touching_length * TOUCHING_BONUS +
        cached_metrics.total_surface_touching * SURFACE_BONUS -
        overlap_penalty -
        num_boxes * BOX_PENALTY;

    // Temperature-based exploration bonus
    double temperature_factor = T / 2000.0;
    if (T > 800) {
        score += (rand() % 200) * temperature_factor;
    } else if (T > 400) {
        score += (rand() % 100) * temperature_factor;
    }

    return (int)std::clamp(score, (long long)INT_MIN / 2, (long long)INT_MAX / 2);
}

int RectangleFittingProblem::objective_delta(
    const std::vector<RectanglePlacement>& new_solution,
    int changed_rect_idx, int T) {

    // If metrics aren't valid, compute full
    if (!metrics_valid || cached_metrics.last_computed_T != T) {
        return objective(new_solution, T);
    }

    // Delta calculation: compute change caused by moving one rectangle
    const auto& old_rect = current_solution[changed_rect_idx];
    const auto& new_rect = new_solution[changed_rect_idx];

    // If position/rotation/box didn't change, return cached
    if (old_rect.x == new_rect.x && old_rect.y == new_rect.y &&
        old_rect.rotated == new_rect.rotated && old_rect.box_id == new_rect.box_id) {
        return objective(current_solution, T);
    }

    // Create delta metrics
    SolutionMetrics delta_metrics = cached_metrics;

    int old_box = old_rect.box_id;
    int new_box = new_rect.box_id;

    // Remove old rectangle's contribution
    int old_w = old_rect.get_actual_width();
    int old_h = old_rect.get_actual_height();

    // Surface touching (old position)
    if (old_rect.x == 0) delta_metrics.total_surface_touching -= old_h;
    if (old_rect.x + old_w == L) delta_metrics.total_surface_touching -= old_h;
    if (old_rect.y == 0) delta_metrics.total_surface_touching -= old_w;
    if (old_rect.y + old_h == L) delta_metrics.total_surface_touching -= old_w;

    // Remove overlaps with rectangles in old box
    for (size_t j = 0; j < current_solution.size(); ++j) {
        if (j == changed_rect_idx) continue;
        const auto& r2 = current_solution[j];
        if (r2.box_id != old_box) continue;

        int overlap_x1 = std::max(old_rect.x, r2.x);
        int overlap_y1 = std::max(old_rect.y, r2.y);
        int overlap_x2 = std::min(old_rect.x + old_w, r2.x + r2.get_actual_width());
        int overlap_y2 = std::min(old_rect.y + old_h, r2.y + r2.get_actual_height());

        if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
            delta_metrics.total_overlap_area -=
                (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
        }

        if (old_rect.adjacent(r2)) {
            if (old_rect.x + old_w == r2.x || r2.x + r2.get_actual_width() == old_rect.x) {
                delta_metrics.total_touching_length -= std::min(old_h, r2.get_actual_height());
            }
            if (old_rect.y + old_h == r2.y || r2.y + r2.get_actual_height() == old_rect.y) {
                delta_metrics.total_touching_length -= std::min(old_w, r2.get_actual_width());
            }
        }
    }

    // Add new rectangle's contribution
    int new_w = new_rect.get_actual_width();
    int new_h = new_rect.get_actual_height();

    // Surface touching (new position)
    if (new_rect.x == 0) delta_metrics.total_surface_touching += new_h;
    if (new_rect.x + new_w == L) delta_metrics.total_surface_touching += new_h;
    if (new_rect.y == 0) delta_metrics.total_surface_touching += new_w;
    if (new_rect.y + new_h == L) delta_metrics.total_surface_touching += new_w;

    // Add overlaps with rectangles in new box
    for (size_t j = 0; j < new_solution.size(); ++j) {
        if (j == changed_rect_idx) continue;
        const auto& r2 = new_solution[j];
        if (r2.box_id != new_box) continue;

        int overlap_x1 = std::max(new_rect.x, r2.x);
        int overlap_y1 = std::max(new_rect.y, r2.y);
        int overlap_x2 = std::min(new_rect.x + new_w, r2.x + r2.get_actual_width());
        int overlap_y2 = std::min(new_rect.y + new_h, r2.y + r2.get_actual_height());

        if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
            delta_metrics.total_overlap_area +=
                (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
        }

        if (new_rect.adjacent(r2)) {
            if (new_rect.x + new_w == r2.x || r2.x + r2.get_actual_width() == new_rect.x) {
                delta_metrics.total_touching_length += std::min(new_h, r2.get_actual_height());
            }
            if (new_rect.y + new_h == r2.y || r2.y + r2.get_actual_height() == new_rect.y) {
                delta_metrics.total_touching_length += std::min(new_w, r2.get_actual_width());
            }
        }
    }

    // Update box area
    if (old_box == new_box) {
        // No box change, area stays same
    } else {
        // Remove from old box
        delta_metrics.box_area_used[old_box] -= (long long)old_rect.width * old_rect.height;
        if (delta_metrics.box_area_used[old_box] == 0) {
            delta_metrics.box_area_used.erase(old_box);
            delta_metrics.boxes_used.erase(old_box);
        }

        // Add to new box
        delta_metrics.box_area_used[new_box] += (long long)new_rect.width * new_rect.height;
        delta_metrics.boxes_used.insert(new_box);
    }

    // Now compute objective using delta metrics
    const int BOX_PENALTY = 1000000;
    const int TOUCHING_BONUS = 3;
    const int SURFACE_BONUS = 1;

    int num_boxes = delta_metrics.boxes_used.size();
    long long box_capacity = (long long)L * L;

    long long utilization_score = 0;
    for (int box_id : delta_metrics.boxes_used) {
        double area_used = (double)delta_metrics.box_area_used[box_id];
        double utilization = area_used / box_capacity;

        if (utilization > 1.0) {
            utilization = 0.0;
        }

        utilization_score += (long long)(utilization * utilization * 14000);
    }

    int overlap_penalty_per_unit = 10;
    if (T < 1800) overlap_penalty_per_unit = 20;
    if (T < 1600) overlap_penalty_per_unit = 50;
    if (T < 1400) overlap_penalty_per_unit = 100;
    if (T < 1200) overlap_penalty_per_unit = 200;
    if (T < 1000) overlap_penalty_per_unit = 500;
    if (T < 800) overlap_penalty_per_unit = 1000;
    if (T < 600) overlap_penalty_per_unit = 2000;
    if (T < 400) overlap_penalty_per_unit = 5000;
    if (T < 200) overlap_penalty_per_unit = 10000;
    if (T < 100) overlap_penalty_per_unit = 50000;
    if (T < 50) overlap_penalty_per_unit = 1000000;

    long long overlap_penalty = delta_metrics.total_overlap_area * overlap_penalty_per_unit;

    long long score =
        utilization_score +
        delta_metrics.total_touching_length * TOUCHING_BONUS +
        delta_metrics.total_surface_touching * SURFACE_BONUS -
        overlap_penalty -
        num_boxes * BOX_PENALTY;

    double temperature_factor = T / 2000.0;
    if (T > 800) {
        score += (rand() % 200) * temperature_factor;
    } else if (T > 400) {
        score += (rand() % 100) * temperature_factor;
    }

    return (int)std::clamp(score, (long long)INT_MIN / 2, (long long)INT_MAX / 2);
}

int RectangleFittingProblem::objective2(const std::vector<RectanglePlacement> &current_solution) {
    return objective(current_solution);
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

bool RectangleFittingProblem::edges_touching(const RectanglePlacement& r1, const RectanglePlacement& r2) {
    if (r1.collides(r2)) return false;

    int w1 = r1.get_actual_width(), h1 = r1.get_actual_height();
    int w2 = r2.get_actual_width(), h2 = r2.get_actual_height();

    bool vertical = (r1.x + w1 == r2.x || r2.x + w2 == r1.x) &&
                    !(r1.y + h1 <= r2.y || r2.y + h2 <= r1.y);
    bool horizontal = (r1.y + h1 == r2.y || r2.y + h2 == r1.y) &&
                      !(r1.x + w1 <= r2.x || r2.x + w2 <= r1.x);

    return vertical || horizontal;
}