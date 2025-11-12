//
// Created by arda on 05.11.25.
//

#ifndef OPTIMALGO_SOLVER_H
#define OPTIMALGO_SOLVER_H


class Solver {
public:
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns, int max_rectangle_in_subproblem);

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int max_steps, int reruns_left);

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem);

    std::vector<RectanglePlacement> solve_one_step_recursive(RectangleFittingProblem &problem);

    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>> splitRectanglesByBoxId(const std::vector<RectanglePlacement>& placements) {
        std::vector<RectanglePlacement> array1, array2;

        if (placements.empty()) {
            return {array1, array2};
        }

        // Group placements by box_id
        std::unordered_map<int, std::vector<RectanglePlacement>> boxes;
        for (const auto& placement : placements) {
            boxes[placement.box_id].push_back(placement);
        }

        // Convert to vector for deterministic ordering (optional)
        std::vector<std::pair<int, std::vector<RectanglePlacement>>> boxVector(boxes.begin(), boxes.end());

        // Sort by box_id for deterministic results (optional)
        std::sort(boxVector.begin(), boxVector.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // Alternate assignment to balance the arrays
        for (size_t i = 0; i < boxVector.size(); ++i) {
            if (i % 2 == 0) {
                // Add all placements from this box to array1
                array1.insert(array1.end(), boxVector[i].second.begin(), boxVector[i].second.end());
            } else {
                // Add all placements from this box to array2
                array2.insert(array2.end(), boxVector[i].second.begin(), boxVector[i].second.end());
            }
        }

        return {array1, array2};
    }


private:
    std::vector<std::vector<RectanglePlacement>> construct_neighbors(RectangleFittingProblem &problem);

};

#endif //OPTIMALGO_SOLVER_H