#ifndef RECTANGLEFITTINGPROBLEM_H
#define RECTANGLEFITTINGPROBLEM_H

#include "OptimizationProblem.h"
#include "RectanglePlacement.h"
#include <vector>

class RectangleFittingProblem : public OptimizationProblem {
private:
    std::vector<RectanglePlacement> rectangles;
    int L;  // Box length

public:
    RectangleFittingProblem(int L, const std::vector<RectanglePlacement>& rectangles);

    int objective(const Input& input) override;

    bool check_all_rectangles_placed(const Input& input) const;
    bool check_no_overlaps(const Input& input) const;
    bool check_within_boxes(const Input& input) const;


    static bool boxes_overlap(const RectanglePlacement& a);

    // Getter for rectangles
    const std::vector<RectanglePlacement>& get_rectangles() const { return rectangles; }

    // Getter for box length
    int get_box_length() const { return L; }
};

#endif // RECTANGLEFITTINGPROBLEM_H