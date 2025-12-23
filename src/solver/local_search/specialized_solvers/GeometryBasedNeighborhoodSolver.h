#ifndef GEOMETRYBASEDNEIGHBORHOODSOLVER_H
#define GEOMETRYBASEDNEIGHBORHOODSOLVER_H

#include "../../../problem/RectangleFittingProblem.h"
#include <vector>
#include <unordered_map>
#include <set>
#include "../LocalSearchSolver.h"

class GeometryBasedNeighborhoodSolver : public LocalSearchSolver {
public:
    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T);

    std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) override;

private:
    struct OccupancyData {
        std::unordered_map<int, std::vector<bool>> grids;
        std::unordered_map<int, int> counts;
        std::set<int> boxes;
    };

    OccupancyData build_occupancy_data(
        const std::vector<RectanglePlacement>& solution, int L);

    bool check_placement(
        const RectanglePlacement& rect,
        int new_x, int new_y, bool new_rotated, int target_box,
        const std::vector<bool>& grid, int L) const;

    std::vector<int> get_sorted_rect_indices(
        const std::vector<RectanglePlacement>& solution,
        const std::unordered_map<int, int>& box_counts) const;


    std::vector<int> get_sorted_boxes(
        const std::set<int>& used_boxes,
        const std::unordered_map<int, int>& box_counts) const;

    void generate_cross_box_neighbors(
        std::vector<std::vector<RectanglePlacement>>& neighbors,
        const std::vector<RectanglePlacement>& solution,
        const std::vector<int>& rect_indices,
        const std::vector<int>& target_boxes,
        const std::function<bool(int, int, int, bool, int)>& can_place,
        int max_neighbors);


    bool try_adjacent_placements(
        std::vector<std::vector<RectanglePlacement>>& neighbors,
        const std::vector<RectanglePlacement>& solution,
        int rect_idx, int target_box,
        const std::function<bool(int, int, int, bool, int)>& can_place,
        int max_neighbors);

    void generate_shift_neighbors(
        std::vector<std::vector<RectanglePlacement>>& neighbors,
        const std::vector<RectanglePlacement>& solution,
        const std::vector<int>& rect_indices,
        const std::function<bool(int, int, int, bool, int)>& can_place,
        int L, int max_neighbors);

    void add_shift_neighbor(
        std::vector<std::vector<RectanglePlacement>>& neighbors,
        const std::vector<RectanglePlacement>& solution,
        int rect_idx, int dx, int dy,
        const std::function<bool(int, int, int, bool, int)>& can_place,
        int L);
};

#endif