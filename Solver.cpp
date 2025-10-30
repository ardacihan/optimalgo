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
    std::vector<RectanglePlacement> solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();

    auto within = [&](const RectanglePlacement& r) {
        return r.x >= 0 && r.y >= 0 &&
               r.x + r.get_actual_width() <= L &&
               r.y + r.get_actual_height() <= L;
    };

    auto collide_fast = [&](const RectanglePlacement& r, int b, int ex) {
        for (int i = 0; i < n; i++) {
            if (i == ex) continue;
            const auto& o = solution[i];
            if (o.box_id != b) continue;
            if (r.x >= o.x + o.get_actual_width() ||
                r.x + r.get_actual_width() <= o.x ||
                r.y >= o.y + o.get_actual_height() ||
                r.y + r.get_actual_height() <= o.y)
                continue;
            return true;
        }
        return false;
    };

    auto box_area = [&](int b) {
        long long a = 0;
        for (auto& r : solution)
            if (r.box_id == b)
                a += r.get_actual_width() * r.get_actual_height();
        return a;
    };

    std::unordered_set<int> boxes;
    for (auto& r : solution) boxes.insert(r.box_id);

    std::vector<int> isolated;
    for (int i = 0; i < n; i++) {
        bool touch = false;
        for (int j = 0; j < n && !touch; j++) {
            if (i == j) continue;
            if (solution[i].box_id != solution[j].box_id) continue;
            if (problem.edges_touching(solution[i], solution[j])) touch = true;
        }
        if (!touch) isolated.push_back(i);
    }

    int max_neighbors = std::min((int)isolated.size(), 200); // cap
    for (int c = 0; c < max_neighbors; ++c) {
        int idx = isolated[c];
        for (int j = 0; j < n; j++) {
            if (solution[j].box_id == solution[idx].box_id) continue;
            RectanglePlacement moved = solution[idx];
            moved.box_id = solution[j].box_id;
            moved.x = solution[j].x;
            moved.y = solution[j].y + solution[j].get_actual_height();
            if (!within(moved) || collide_fast(moved, moved.box_id, idx)) continue;
            auto nb = solution;
            nb[idx] = moved;
            nbs.push_back(std::move(nb));
            if (nbs.size() > 500) return nbs; // early cutoff
        }
    }

    std::vector<std::pair<int, long long>> box_area_vec;
    box_area_vec.reserve(boxes.size());
    for (int b : boxes) box_area_vec.push_back({b, box_area(b)});
    std::sort(box_area_vec.begin(), box_area_vec.end(), [](auto& a, auto& b) { return a.second < b.second; });
    if (box_area_vec.size() < 2) return nbs;

    int from = box_area_vec.front().first;
    int to = box_area_vec.back().first;
    for (int i = 0; i < n; i++) {
        if (solution[i].box_id != from) continue;
        RectanglePlacement moved = solution[i];
        moved.box_id = to;
        moved.x = 0;
        moved.y = 0;
        if (!within(moved) || collide_fast(moved, moved.box_id, i)) continue;
        auto nb = solution;
        nb[i] = moved;
        nbs.push_back(std::move(nb));
        if (nbs.size() > 1000) break; // cap for speed
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

