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
    // SIMPLIFIED PARAMETERS
    const int BOX_PENALTY = 10000;      // Dominant penalty per box
    const int OVERLAP_PENALTY = 50;     // Per unit area
    const int UTILIZATION_BONUS = 300;  // Reward for high utilization
    const int TOUCHING_OTHER_BONUS = 5; // Per unit length of touching other rectangles
    const int TOUCHING_SURFACE_BONUS = 2; // Per unit length of touching box boundaries

    if (current_solution.empty()) return 0;

    std::unordered_set<int> boxes_used;
    std::unordered_map<int, long long> box_area_used;
    long long total_overlap_area = 0;
    long long total_touching_other_length = 0;
    long long total_touching_surface_length = 0;

    // 1. Calculate box usage, overlaps, and touching lengths
    for (size_t i = 0; i < current_solution.size(); ++i) {
        const auto& r1 = current_solution[i];
        boxes_used.insert(r1.box_id);
        box_area_used[r1.box_id] += (long long)r1.get_actual_width() * r1.get_actual_height();

        int w1 = r1.get_actual_width();
        int h1 = r1.get_actual_height();

        // Check for touching box boundaries (surface touching)
        // Left edge touches box boundary
        if (r1.x == 0) total_touching_surface_length += h1;
        // Right edge touches box boundary
        if (r1.x + w1 == L) total_touching_surface_length += h1;
        // Bottom edge touches box boundary
        if (r1.y == 0) total_touching_surface_length += w1;
        // Top edge touches box boundary
        if (r1.y + h1 == L) total_touching_surface_length += w1;

        // Check overlaps and adjacencies with other rectangles
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            const auto& r2 = current_solution[j];
            if (r1.box_id != r2.box_id) continue;

            // Check for overlap
            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + w1, r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + h1, r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                total_overlap_area += (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }

            // Check for touching/adjacency (rectangle-to-rectangle touching)
            if (r1.adjacent(r2)) {
                // Calculate touching length for adjacent rectangles
                int w2 = r2.get_actual_width();
                int h2 = r2.get_actual_height();

                int left1 = r1.x;
                int right1 = r1.x + w1;
                int bottom1 = r1.y;
                int top1 = r1.y + h1;

                int left2 = r2.x;
                int right2 = r2.x + w2;
                int bottom2 = r2.y;
                int top2 = r2.y + h2;

                // Vertical adjacency (left/right touching)
                if (right1 == left2 || left1 == right2) {
                    int overlap_top = std::min(top1, top2);
                    int overlap_bottom = std::max(bottom1, bottom2);
                    total_touching_other_length += std::max(0, overlap_top - overlap_bottom);
                }
                // Horizontal adjacency (top/bottom touching)
                else if (top1 == bottom2 || bottom1 == top2) {
                    int overlap_right = std::min(right1, right2);
                    int overlap_left = std::max(left1, left2);
                    total_touching_other_length += std::max(0, overlap_right - overlap_left);
                }
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

    // 3. Final score
    long long score =
        -num_boxes * BOX_PENALTY +    // Fewer boxes is better
        utilization_bonus -           // Higher utilization is better
        total_overlap_area * OVERLAP_PENALTY + // Overlaps are bad
        total_touching_other_length * TOUCHING_OTHER_BONUS + // Rectangle touching is good
        total_touching_surface_length * TOUCHING_SURFACE_BONUS; // Surface touching is also good

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