#ifndef OPTIMALGO_GREEDYSOLVER_H
#define OPTIMALGO_GREEDYSOLVER_H

#include "../../problem/RectangleFittingProblem.h"
#include <vector>
#include <memory>
#include "../../solver/RectangleFittingProblemSolver.h"

class GreedySolver : public RectangleFittingProblemSolver {
public:
    std::vector<RectanglePlacement> solve(RectangleFittingProblem &problem, int num_reruns,
                                          int max_rectangle_in_subproblem, int T) override;

    std::vector<RectanglePlacement> solve_with_reruns(RectangleFittingProblem &problem, int num_reruns,
                                      int max_rectangle_in_subproblem, int T) override;

    // Simple strategy selection
    // 0 = biggest first, 1 = smallest first
    void set_selection_strategy(int strategy_type) {
        current_strategy = strategy_type;
    }

private:
    int current_strategy = 0;

    // Main solve functions
    std::vector<RectanglePlacement> solve_biggest_first(RectangleFittingProblem &problem);
    std::vector<RectanglePlacement> solve_smallest_first(RectangleFittingProblem &problem);

    // Placement utilities
    RectanglePlacement place_rectangle(RectangleFittingProblem &problem,
                                      const RectanglePlacement &selected_rect,
                                      std::vector<std::vector<int>>& occupancy_grids,
                                      int& next_box_id);

    bool collides_with_occupancy(int x, int y, int w, int h,
                                const std::vector<int>& grid, int L);

    void mark_occupied(const RectanglePlacement& placement,
                      std::vector<std::vector<int>>& occupancy_grids,
                      int rect_idx, int L);

    std::vector<RectanglePlacement> solve_best_fit(RectangleFittingProblem &problem);

    int calculate_fit_score(int x, int y, int w, int h, const std::vector<int> &grid, int L);
};

#endif //OPTIMALGO_GREEDYSOLVER_H