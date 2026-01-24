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

    // For step-by-step visualization
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T);

    // Simple strategy selection
    // 0 = biggest first, 1 = smallest first
    void set_selection_strategy(int strategy_type) {
        current_strategy = strategy_type;
    }

    void reset_state() {
        step_index = 0;
        step_placements.clear();
        occupancy_grids.clear();
        next_box_id = 0;
    }

private:
    int current_strategy = 0;
    
    // Step-by-step state
    int step_index = 0;
    std::vector<RectanglePlacement> step_placements;
    std::vector<std::vector<int>> occupancy_grids;
    int next_box_id = 0;

    // Main solve functions
    std::vector<RectanglePlacement> solve_biggest_first(RectangleFittingProblem &problem);
    std::vector<RectanglePlacement> solve_smallest_first(RectangleFittingProblem &problem);
    std::vector<RectanglePlacement> solve_best_fit(RectangleFittingProblem &problem);

    // Step-by-step placement
    RectanglePlacement place_rectangle_step(RectangleFittingProblem &problem,
                                          const RectanglePlacement &selected_rect);
    
    // Complete placement (original)
    RectanglePlacement place_rectangle(RectangleFittingProblem &problem,
                                      const RectanglePlacement &selected_rect,
                                      std::vector<std::vector<int>>& occupancy_grids,
                                      int& next_box_id);

    bool collides_with_occupancy(int x, int y, int w, int h,
                                const std::vector<int>& grid, int L);

    void mark_occupied(const RectanglePlacement& placement,
                      std::vector<std::vector<int>>& occupancy_grids,
                      int rect_idx, int L);

    int calculate_fit_score(int x, int y, int w, int h, const std::vector<int> &grid, int L);
};

#endif //OPTIMALGO_GREEDYSOLVER_H