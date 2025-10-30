#include "RectangleFittingProblem.h"
#include <algorithm>
#include <complex>
#include <map>
#include <iostream>


int RectangleFittingProblem::objective(const std::vector<RectanglePlacement>& current_solution) {
    const int BIG = 1'000'000;
    const int PENALTY = 10'000;
    const int TOUCH_BONUS = 500;

    std::unordered_set<int> boxes;
    for (auto& r : current_solution) boxes.insert(r.box_id);
    int num_boxes = boxes.size();

    long long overlap = 0, touch_bonus = 0;
    for (size_t i = 0; i < current_solution.size(); ++i) {
        for (size_t j = i + 1; j < current_solution.size(); ++j) {
            if (current_solution[i].box_id != current_solution[j].box_id) continue;
            overlap += current_solution[i].getOverlapArea(current_solution[j]);
            if (edges_touching(current_solution[i], current_solution[j])) touch_bonus++;
        }
    }

    std::unordered_map<int, long long> cov;
    long long unused = 0;
    for (auto& r : current_solution)
        cov[r.box_id] += r.get_actual_width() * r.get_actual_height();
    for (int b : boxes)
        unused += (1LL * L * L - cov[b]);

    long long score = -(long long)num_boxes * BIG;
    score -= overlap * PENALTY;
    score -= unused / 100;
    score += touch_bonus * TOUCH_BONUS;

    if (score > INT_MAX) return INT_MAX;
    if (score < INT_MIN) return INT_MIN;
    return (int)score;
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

bool RectangleFittingProblem::solution_legal(const std::vector<RectanglePlacement>& current_solution)
    const{ return check_no_overlaps(current_solution) && check_within_boxes(current_solution); };

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