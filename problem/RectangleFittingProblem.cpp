#include "RectangleFittingProblem.h"
#include <algorithm>
#include <complex>
#include <map>
#include <climits>
#include <cmath>

int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& current_solution) {
    return objective(current_solution, 1000);
}

int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& current_solution, int T) {
    // SIMPLIFIED PARAMETERS - focused on what matters
    const int BOX_PENALTY = 10000;           // Strong penalty per box (minimize boxes)
    const int TOUCHING_BONUS = 5;           // Bonus for rectangle-to-rectangle touching
    const int SURFACE_BONUS = 2;            // Bonus for touching box boundaries

    if (current_solution.empty()) return 0;

    std::unordered_set<int> boxes_used;
    std::unordered_map<int, long long> box_area_used;
    long long total_overlap_area = 0;
    long long total_touching_length = 0;
    long long total_surface_touching = 0;

    // 1. Calculate basic metrics
    for (size_t i = 0; i < current_solution.size(); ++i) {
        const auto& r1 = current_solution[i];
        int box_id = r1.box_id;
        boxes_used.insert(box_id);

        // Add rectangle area to box total (NOT using actual placed area for overlaps)
        box_area_used[box_id] += (long long)r1.width * r1.height; // Use original dimensions

        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        // Surface touching
        if (r1.x == 0) total_surface_touching += h1;
        if (r1.x + w1 == L) total_surface_touching += h1;
        if (r1.y == 0) total_surface_touching += w1;
        if (r1.y + h1 == L) total_surface_touching += w1;

        // Check overlaps and adjacencies with other rectangles
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& r2 = current_solution[j];
            if (box_id != r2.box_id) continue;

            // Overlap calculation
            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + w1, r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + h1, r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                total_overlap_area += (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }

            // Rectangle-to-rectangle touching (adjacency)
            if (r1.adjacent(r2)) {
                // Simplified: count touching edges
                if (r1.x + w1 == r2.x || r2.x + r2.get_actual_width() == r1.x) {
                    total_touching_length += std::min(h1, r2.get_actual_height());
                }
                if (r1.y + h1 == r2.y || r2.y + r2.get_actual_height() == r1.y) {
                    total_touching_length += std::min(w1, r2.get_actual_width());
                }
            }
        }
    }

    int num_boxes = boxes_used.size();
    long long box_capacity = (long long)L * L;

    // 2. Calculate utilization score with 100% cap
    long long utilization_score = 0;
    for (int box_id : boxes_used) {
        double area_used = (double)box_area_used[box_id];
        double utilization = area_used / box_capacity;

        // CRITICAL: Don't reward >100% utilization (means overlaps)
        if (utilization > 1.0) {
            utilization = 0.0; // Dont reward if exceeding
        }

        // Reward high utilization, but not over 100%
        // Quadratic reward: 50% util = 0.25, 100% util = 1.0
        utilization_score += (long long)(utilization * utilization * 1000);
    }

    // 3. TEMPERATURE-DEPENDENT OVERLAP PENALTY
    // This is the key for simulated annealing:
    // - At T=1000: Small or zero penalty (allow overlaps for exploration)
    // - At T=0: High penalty (force overlap resolution)

    double temperature_factor = T / 1000.0; // 1.0 at T=1000, 0.0 at T=0

    // Dynamic penalty: starts at 0, increases to 10 as T decreases
    int overlap_penalty_per_unit = 0;
    if (T < 800) overlap_penalty_per_unit = 100;
    if (T < 600) overlap_penalty_per_unit = 200;
    if (T < 400) overlap_penalty_per_unit = 300;
    if (T < 200) overlap_penalty_per_unit = 4000;
    if (T < 50)  overlap_penalty_per_unit = 100000; // Very strict near the end

    long long overlap_penalty = total_overlap_area * overlap_penalty_per_unit;

    // 4. Calculate final score
    // We want to MAXIMIZE this score
    long long score =
        utilization_score +                     // Higher utilization is better
        total_touching_length * TOUCHING_BONUS + // Touching rectangles is good
        total_surface_touching * SURFACE_BONUS - // Touching boundaries is okay
        overlap_penalty -                       // Overlaps are bad (temp-dependent)
        num_boxes * BOX_PENALTY;                // Fewer boxes is MUCH better

    // 5. Temperature-based exploration bonus (encourage trying different box counts)
    // At high T, we want to explore different numbers of boxes
    if (T > 500) {
        // Add small random component to avoid getting stuck
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