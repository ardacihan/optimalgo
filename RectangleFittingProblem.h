//
// RectangleFittingProblem.h
//

#ifndef RECTANGLEFITTINGPROBLEM_H
#define RECTANGLEFITTINGPROBLEM_H

#include "OptimizationProblem.h"
#include "RectanglePlacement.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

class RectangleFittingProblem : public OptimizationProblem<std::vector<RectanglePlacement>> {
private:
    int L; // Box length
    std::vector<RectanglePlacement> current_solution;

    bool check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const;
    bool check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const;

public:
    RectangleFittingProblem(int L, const std::vector<RectanglePlacement>& initial_solution)
        : L(L), current_solution(initial_solution) {}

    int objective(const std::vector<RectanglePlacement>& current_solution) override;

    int objective2(const std::vector<RectanglePlacement> &current_solution);

    bool solution_legal(const std::vector<RectanglePlacement>& current_solution) const override;

    std::vector<RectanglePlacement> get_current_solution() const override {
        return current_solution;
    }

    void set_current_solution(const std::vector<RectanglePlacement>& solution) override {
        current_solution = solution;
    }

    int get_box_length() const { return L; }

    std::unordered_map<int, int> get_coverage_each_bounding_box() const {
        std::unordered_map<int, int> coverage;
        for (const auto& rect : current_solution) {
            coverage[rect.box_id] += rect.get_actual_width() * rect.get_actual_height();
        }
        return coverage;
    }

    static bool edges_touching(const RectanglePlacement& r1, const RectanglePlacement& r2);

};

#endif // RECTANGLEFITTINGPROBLEM_H
