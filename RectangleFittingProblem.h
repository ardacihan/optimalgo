#ifndef RECTANGLEFITTINGPROBLEM_H
#define RECTANGLEFITTINGPROBLEM_H

#include "OptimizationProblem.h"
#include "RectanglePlacement.h"
#include <vector>
#include <memory>

class RectangleFittingProblem : public OptimizationProblem {
private:
    int L;  // Box length
    std::vector<RectanglePlacement> current_solution;

public:
    RectangleFittingProblem(int L, std::vector<RectanglePlacement>& current_solution) {
        bool a = check_no_overlaps(current_solution);
        bool b = check_within_boxes(current_solution);
        if (a && b) {
            this->L = L;
            this->current_solution = current_solution;
        };
    };

    int objective(const std::vector<RectanglePlacement>& input);

    bool check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const;
    bool check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const;

    std::vector<RectanglePlacement> get_rectangles() const { return current_solution; }

    static bool edges_touching(const RectanglePlacement r1, const RectanglePlacement r2);

    int calculate_cover_area(std::vector<RectanglePlacement> current_solution);


    // Getter for box length
    int get_box_length() const { return L; }
};

#endif // RECTANGLEFITTINGPROBLEM_H