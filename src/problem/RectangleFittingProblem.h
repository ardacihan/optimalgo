// ============================================================================
// FILE: problem/RectangleFittingProblem.h (UPDATED)
// ============================================================================
#ifndef RECTANGLEFITTINGPROBLEM_H
#define RECTANGLEFITTINGPROBLEM_H

#include "OptimizationProblem.h"
#include "primitives/RectanglePlacement.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>

struct SolutionMetrics {
    std::unordered_set<int> boxes_used;
    std::unordered_map<int, long long> box_area_used;
    long long total_overlap_area;
    long long total_touching_length;
    long long total_surface_touching;
    int last_computed_T;

    SolutionMetrics() : total_overlap_area(0), total_touching_length(0),
                       total_surface_touching(0), last_computed_T(-1) {}
};

class RectangleFittingProblem : public OptimizationProblem<std::vector<RectanglePlacement>> {
private:
    int L;
    std::vector<RectanglePlacement> current_solution;
    bool metrics_valid;

    bool check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const;
    bool check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const;

    void compute_full_metrics(const std::vector<RectanglePlacement>& solution, int T);

public:
    RectangleFittingProblem(int L, const std::vector<RectanglePlacement>& initial_solution)
        : L(L), current_solution(initial_solution), metrics_valid(false) {}

    double objective(const std::vector<RectanglePlacement>& current_solution) override;

    double objective(const std::vector<RectanglePlacement>& current_solution, int T);

    int objective_delta(const std::vector<RectanglePlacement>& new_solution,
                       int changed_rect_idx, int T);

    bool solution_legal(const std::vector<RectanglePlacement>& current_solution) const override;

    std::vector<RectanglePlacement> get_current_solution() const override {
        return current_solution;
    }

    void set_current_solution(const std::vector<RectanglePlacement>& solution) override {
        current_solution = solution;
        metrics_valid = false; // Invalidate cache
    }

    int get_box_length() const { return L; }

    void invalidate_metrics() { metrics_valid = false; }

    std::unordered_map<int, int> get_coverage_each_bounding_box() const {
        std::unordered_map<int, int> coverage;
        for (const auto& rect : current_solution) {
            coverage[rect.box_id] += rect.get_actual_width() * rect.get_actual_height();
        }
        return coverage;
    }

    static bool edges_touching(const RectanglePlacement& r1, const RectanglePlacement& r2);
};

#endif