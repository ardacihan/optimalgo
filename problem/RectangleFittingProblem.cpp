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
    // SIMPLIFIED PARAMETERS - Less is more
    const int BOX_PENALTY = 10000;      // Dominant penalty per box
    const int OVERLAP_PENALTY = 50;     // Per unit area
    const int UTILIZATION_BONUS = 300;  // Reward for high utilization

    if (current_solution.empty()) return 0;

    std::unordered_set<int> boxes_used;
    std::unordered_map<int, long long> box_area_used;
    long long total_overlap_area = 0;

    // 1. Calculate box usage and overlaps
    for (size_t i = 0; i < current_solution.size(); ++i) {
        const auto& r1 = current_solution[i];
        boxes_used.insert(r1.box_id);
        box_area_used[r1.box_id] += (long long)r1.get_actual_width() * r1.get_actual_height();

        // Check overlaps with other rectangles in same box
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& r2 = current_solution[j];
            if (r1.box_id != r2.box_id) continue;

            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + r1.get_actual_width(), r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + r1.get_actual_height(), r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                total_overlap_area += (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }
        }
    }

    int num_boxes = boxes_used.size();
    long long box_capacity = (long long)L * L;

    // 2. Calculate utilization bonus (convex reward)
    double utilization_bonus = 1.0;
    for (int box_id : boxes_used) {
        double util = (double)box_area_used[box_id] / box_capacity;
        // Convex reward: util^2 encourages consolidation
        utilization_bonus += util * util * UTILIZATION_BONUS;
    }

    // 3. Final score - SIMPLIFIED
    long long score =
        -num_boxes * BOX_PENALTY +    // Fewer boxes is better
        utilization_bonus -           // Higher utilization is better
        total_overlap_area * OVERLAP_PENALTY; // Overlaps are bad

    // 4. Add temperature-based escape from local optima
    if (T > 0) {
        // At high temperatures, reduce the box penalty to allow exploring more boxes
        double temp_factor = std::min(1.0, T / 1000.0);
        score += num_boxes * BOX_PENALTY * 0.1 * temp_factor; // Temporary relief
    }

    return (int)std::clamp(score, (long long)INT_MIN, (long long)INT_MAX);
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