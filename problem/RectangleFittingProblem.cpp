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
    // PARAMETERS
    // BIG: The reward for removing a box entirely. Must be the dominant factor.
    const int BIG = 1000000;
    const int PENALTY = 5000;
    const int TOUCH_BONUS = 5;       // Reduced slightly so it doesn't prevent moving
    const int SPARSE_BOX_PENALTY = 200;

    // CRITICAL CHANGE: Use Square (2.0) instead of 5.0.
    // This creates a convex curve where 2x 50% boxes are worse than 1x 100% box.
    const double UTIL_REWARD_EXP = 2.0;
    const double UTIL_MULTIPLIER = 100000.0;

    if (current_solution.empty()) return 0;

    std::unordered_set<int> boxes;
    std::unordered_map<int, long long> box_area_used;
    std::unordered_map<int, int> box_item_count;

    // 1. Calculate Basics & Overlaps
    long long overlap_penalty = 0;
    long long touch_bonus = 0;

    for (size_t i = 0; i < current_solution.size(); ++i) {
        const auto& r1 = current_solution[i];

        // Track box stats
        boxes.insert(r1.box_id);
        box_area_used[r1.box_id] += (long long)r1.get_actual_width() * r1.get_actual_height();
        box_item_count[r1.box_id]++;

        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& r2 = current_solution[j];

            // Only care about interactions in the same box
            if (r1.box_id != r2.box_id) continue;

            // Overlap Calculation
            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + r1.get_actual_width(), r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + r1.get_actual_height(), r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                long long overlap_area = (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);

                // Dynamic penalty based on Temperature (if T is high, penalty is lower to allow traversal)
                double temp_factor = 1.0;
                if (T < 1000) temp_factor = 1.0 - (T / 1200.0); // Simple linear decay

                overlap_penalty += (long long)(overlap_area * PENALTY * temp_factor);
            }

            // Adjacency Bonus
            if (edges_touching(r1, r2)) {
                touch_bonus += TOUCH_BONUS;
            }
        }
    }

    int num_boxes = boxes.size();
    long long box_capacity = (long long)L * L;
    double utilization_score = 0.0;
    long long sparse_penalty = 0;

    // 2. Calculate Sum of Squares Score
    for (int b_id : boxes) {
        double util = (double)box_area_used[b_id] / (double)box_capacity;

        // REWARD: Utilization^2
        // Example:
        // Two boxes at 50% = 0.25 + 0.25 = 0.5 score
        // One box at 100%  = 1.0 score (Better!)
        utilization_score += std::pow(util, UTIL_REWARD_EXP) * UTIL_MULTIPLIER;

        // Penalty for extremely sparse boxes (e.g. < 10% full) to encourage emptying them
        if (util < 0.10) {
            sparse_penalty += SPARSE_BOX_PENALTY;
        }
    }

    // 3. Final Assembly
    // Base score is negative (minimize boxes)
    long long score = -(long long)num_boxes * BIG;

    score += (long long)utilization_score; // Add the convex reward
    score += touch_bonus;
    score -= overlap_penalty;
    score -= sparse_penalty;

    // Safety clamping
    if (score > INT_MAX) return INT_MAX;
    if (score < INT_MIN) return INT_MIN;

    return (int)score;
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