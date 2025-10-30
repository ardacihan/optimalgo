//
// Created by ardac on 28/10/2025.
//

#include "Solver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>

// get each rectangle
// move them around
// move them from a box to another box
// check constraints


// reward if total sum of boxes are better -> this is by choosing the neighbors
// also reward them if moving them closer to other  -> this is by choosing the neighbors
std::vector<std::vector<RectanglePlacement>> GeometryBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {
    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();

    auto within = [&](const RectanglePlacement& r) {
        return r.x >= 0 && r.y >= 0 &&
               r.x + r.get_actual_width() <= L &&
               r.y + r.get_actual_height() <= L;
    };

    auto collide = [&](const RectanglePlacement& r, int b, int ex) {
        for (int i = 0; i < n; i++) {
            if (i == ex) continue;
            const auto& o = solution[i];
            if (o.box_id != b) continue;
            if (r.collides(o)) return true;
        }
        return false;
    };

    auto area = [&](int b) {
        long long a = 0;
        for (auto& r : solution)
            if (r.box_id == b)
                a += r.get_actual_width() * r.get_actual_height();
        return a;
    };

    std::set<int> boxes;
    for (auto& r : solution)
        boxes.insert(r.box_id);

    std::vector<int> isolated;
    for (int i = 0; i < n; i++) {
        bool touch = false;
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            if (solution[i].box_id != solution[j].box_id) continue;
            if (problem.edges_touching(solution[i], solution[j])) {
                touch = true;
                break;
            }
        }
        if (!touch) isolated.push_back(i);
    }

    for (int idx : isolated) {
        for (int j = 0; j < n; j++) {
            if (solution[j].box_id == solution[idx].box_id) continue;
            RectanglePlacement moved = solution[idx];
            moved.box_id = solution[j].box_id;

            std::vector<std::pair<int,int>> pos = {
                {solution[j].x + solution[j].get_actual_width(), solution[j].y},
                {solution[j].x, solution[j].y + solution[j].get_actual_height()},
                {solution[j].x, solution[j].y - moved.get_actual_height()}
            };

            for (auto [nx, ny] : pos) {
                moved.x = nx;
                moved.y = ny;
                if (within(moved) && !collide(moved, moved.box_id, idx)) {
                    auto nb = solution;
                    nb[idx] = moved;
                    nbs.push_back(nb);
                }
            }
        }
    }

    std::vector<std::pair<int, long long>> box_area;
    for (int b : boxes)
        box_area.push_back({b, area(b)});
    std::sort(box_area.begin(), box_area.end(), [](auto& a, auto& b) {
        return a.second < b.second;
    });

    if (box_area.size() >= 2) {
        int from = box_area.front().first;
        int to = box_area.back().first;
        std::vector<int> from_rects;
        for (int i = 0; i < n; i++)
            if (solution[i].box_id == from)
                from_rects.push_back(i);
        for (int idx : from_rects) {
            RectanglePlacement moved = solution[idx];
            moved.box_id = to;
            moved.x = 0;
            moved.y = 0;
            if (within(moved) && !collide(moved, moved.box_id, idx)) {
                auto nb = solution;
                nb[idx] = moved;
                nbs.push_back(nb);
            }
        }
    }

    return nbs;
}


std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve(RectangleFittingProblem &problem, int max_steps) {
    std::vector<RectanglePlacement> solution = problem.get_current_solution();
    if (max_steps <= 0) return solution;

    auto neighbors = construct_neighbors(problem);
    if (neighbors.empty()) return solution;

    // Select neighbor with best objective
    std::vector<RectanglePlacement> next_solution = neighbors[0];
    int best_obj = problem.objective(next_solution);
    for (auto &n : neighbors) {
        int obj = problem.objective(n);
        if (obj > best_obj) {
            best_obj = obj;
            next_solution = n;
        }
    }

    problem.set_current_solution(next_solution);
    std::cout <<"Taking the next step with obj score: " <<best_obj << std::endl;
    return solve(problem, max_steps - 1);
}

