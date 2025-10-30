#ifndef RECTANGLEFITTINGPROBLEM_H
#define RECTANGLEFITTINGPROBLEM_H

#include "OptimizationProblem.h"
#include "RectanglePlacement.h"
#include <vector>
#include <unordered_set>

class RectangleFittingProblem : public OptimizationProblem {
private:
    int L;  // Box length
    std::vector<RectanglePlacement> current_solution;

public:
    RectangleFittingProblem(int L, std::vector<RectanglePlacement>& current_solution) {
        this->L = L;
        bool a = check_no_overlaps(current_solution);
        bool b = check_within_boxes(current_solution);
        if (a && b) {
            this->current_solution = current_solution;
        };
    };

    int objective(const std::vector<RectanglePlacement>& current_solution);

    bool check_no_overlaps(const std::vector<RectanglePlacement>& current_solution) const;
    bool check_within_boxes(const std::vector<RectanglePlacement>& current_solution) const;

    bool solution_legal(const std::vector<RectanglePlacement> &current_solution) const;

    std::vector<RectanglePlacement> get_rectangles() const { return current_solution; }

    static bool edges_touching(const RectanglePlacement r1, const RectanglePlacement r2);


    // Getter for box length
    int get_box_length() const { return L; }

    std::vector<RectanglePlacement> get_current_solution() { return current_solution;}

    void set_current_solution(std::vector<RectanglePlacement> current_solution) {
        this->current_solution = current_solution;
    }

    int get_num_unique_boxes() {
        const std::vector<RectanglePlacement>& solution = get_current_solution();
        std::unordered_set<int> unique_boxes;

        for (const auto& rect : solution) {
            unique_boxes.insert(rect.box_id);
        }

        return static_cast<int>(unique_boxes.size());
    }

    std::unordered_map<int, int> get_coverage_each_bounding_box() {
        const std::vector<RectanglePlacement>& solution = get_current_solution();
        std::unordered_map<int, int> coverage_per_box;

        for (const auto& rect : solution) {
            int box_id = rect.box_id;
            int rect_area = rect.get_actual_width() * rect.get_actual_height();
            coverage_per_box[box_id] += rect_area;
        }

        return coverage_per_box;
    }

    std::vector<std::vector<RectanglePlacement>> group_rectangles_by_bounding_box(
        const std::vector<RectanglePlacement>& solution) {
        std::unordered_map<int, std::vector<RectanglePlacement>> box_groups;

        // Group rectangles by their box_id
        for (const auto& rect : solution) {
            box_groups[rect.box_id].push_back(rect);
        }

        // Convert to vector of vectors
        std::vector<std::vector<RectanglePlacement>> result;
        for (auto& [box_id, rectangles] : box_groups) {
            result.push_back(std::move(rectangles));
        }

        return result;
    }

    std::vector<std::reference_wrapper<const RectanglePlacement>> getPlacementsInSameBoundingBoxRef(int targetBoxId) {

            std::vector<std::reference_wrapper<const RectanglePlacement>> result;

            for (const auto& placement : current_solution) {
                if (placement.box_id == targetBoxId) {
                    result.push_back(std::cref(placement));
                }
            }

            return result;
    }

};

#endif // RECTANGLEFITTINGPROBLEM_H