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

    const int MAX_NEIGHBORS = 300;
    const int MAX_PER_RECT = std::max(1, MAX_NEIGHBORS / n);

    // ---- Minimal validity check ----
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

    // ---- Pure geometry-based neighborhood (capped + fair + traced) ----
    for (int i = 0; i < n && nbs.size() < MAX_NEIGHBORS; i++) {
        const auto& moving = solution[i];

        int old_box = moving.box_id;
        int old_x   = moving.x;
        int old_y   = moving.y;

        int per_rect_count = 0;

        for (int j = 0; j < n && nbs.size() < MAX_NEIGHBORS; j++) {
            if (i == j) continue;

            const auto& anchor = solution[j];

            for (int rot = 0; rot < 2 && nbs.size() < MAX_NEIGHBORS; rot++) {
                if (rot == 1 && moving.width == moving.height) continue;

                int w = (rot == 0) ? moving.width  : moving.height;
                int h = (rot == 0) ? moving.height : moving.width;

                std::vector<std::pair<int,int>> positions = {
                    { anchor.x - w, anchor.y },                                   // left
                    { anchor.x + anchor.get_actual_width(), anchor.y },          // right
                    { anchor.x, anchor.y - h },                                   // bottom
                    { anchor.x, anchor.y + anchor.get_actual_height() }           // top
                };

                for (auto& p : positions) {
                    if (nbs.size() >= MAX_NEIGHBORS) break;
                    if (per_rect_count >= MAX_PER_RECT) break;

                    auto neighbor = solution;
                    neighbor[i].box_id  = anchor.box_id;
                    neighbor[i].x       = p.first;
                    neighbor[i].y       = p.second;
                    neighbor[i].rotated = (rot == 1);

                    if (is_valid_placement(neighbor[i], neighbor, i)) {

                        nbs.push_back(neighbor);
                        per_rect_count++;
                    }
                }
            }
        }
    }

    std::cout << "GeometryBased (MINIMAL + CAPPED + FAIR): Generated "
              << nbs.size() << " neighbors\n";

    return nbs;
}



