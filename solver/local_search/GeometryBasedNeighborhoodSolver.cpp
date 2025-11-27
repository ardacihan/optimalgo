#include "GeometryBasedNeighborhoodSolver.h"
#include "problem/BoxOccupancyUtil.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>
#include <cmath>

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem)
{
    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 100;

    // Group rectangles by box
    std::unordered_map<int, std::vector<int>> box_rects;
    for (int i = 0; i < n; i++) {
        box_rects[solution[i].box_id].push_back(i);
    }

    // Find lone rectangles (only 1 rect in box)
    std::vector<int> lone_rects;
    for (const auto& [box_id, rects] : box_rects) {
        if (rects.size() == 1) {
            lone_rects.push_back(rects[0]);
        }
    }

    if (lone_rects.empty()) return nbs;

    // Validity check
    auto is_valid_placement = [&](const RectanglePlacement& r,
                                  const std::vector<RectanglePlacement>& sol,
                                  int skip_idx)
    {
        if (r.x < 0 || r.y < 0 ||
            r.x + r.get_actual_width() > L ||
            r.y + r.get_actual_height() > L)
            return false;

        for (int i = 0; i < sol.size(); i++) {
            if (i == skip_idx) continue;
            if (sol[i].box_id == r.box_id && r.collides(sol[i]))
                return false;
        }
        return true;
    };

    // Try to move lone rectangles near others
    for (int lone_idx : lone_rects) {
        if (nbs.size() >= MAX_NEIGHBORS) break;

        const auto& lone_rect = solution[lone_idx];

        for (const auto& [target_box_id, rects] : box_rects) {
            if (nbs.size() >= MAX_NEIGHBORS) break;
            if (target_box_id == lone_rect.box_id) continue;

            for (int target_rect_idx : rects) {
                if (nbs.size() >= MAX_NEIGHBORS) break;

                const auto& target_rect = solution[target_rect_idx];

                for (int rot = 0; rot < 2; rot++) {
                    if (rot == 1 && lone_rect.width == lone_rect.height) continue;

                    int w = (rot == 0) ? lone_rect.width  : lone_rect.height;
                    int h = (rot == 0) ? lone_rect.height : lone_rect.width;

                    std::vector<std::pair<int, int>> positions = {
                        {target_rect.x - w, target_rect.y},
                        {target_rect.x + target_rect.get_actual_width(), target_rect.y},
                        {target_rect.x, target_rect.y - h},
                        {target_rect.x, target_rect.y + target_rect.get_actual_height()}
                    };

                    for (const auto& base_pos : positions) {
                        if (nbs.size() >= MAX_NEIGHBORS) break;

                        auto neighbor = solution;
                        neighbor[lone_idx].box_id = target_box_id;
                        neighbor[lone_idx].x = base_pos.first;
                        neighbor[lone_idx].y = base_pos.second;
                        neighbor[lone_idx].rotated = (rot == 1);

                        if (is_valid_placement(neighbor[lone_idx], neighbor, lone_idx)) {
                            nbs.push_back(neighbor);
                        }
                    }
                }
            }
        }
    }

    std::cout << "GeometryBased (OLD): Generated " << nbs.size() << " neighbors\n";
    return nbs;
}
