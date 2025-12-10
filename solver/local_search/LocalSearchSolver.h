#ifndef OPTIMALGO_SOLVER_H
#define OPTIMALGO_SOLVER_H

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <set>
#include <random>
#include <future>
#include <thread>

#include "solver/RectangleFittingProblemSolver.h"

struct NeighborMetadata {
    int changed_rect_idx;
    bool can_use_delta;
    NeighborMetadata() : changed_rect_idx(-1), can_use_delta(false) {}
    NeighborMetadata(int idx) : changed_rect_idx(idx), can_use_delta(true) {}
};

class LocalSearchSolver : public RectangleFittingProblemSolver {
private:
    std::random_device rd;
    std::mt19937 gen;

public:
    LocalSearchSolver() : gen(rd()) {}
    virtual ~LocalSearchSolver() = default;

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem,
          int num_reruns,
          int max_rectangle_in_subproblem,
          int T);

    std::vector<RectanglePlacement>
    solve(RectangleFittingProblem &problem,
          int num_reruns,
          int max_rectangle_in_subproblem,
          int T) override
    {
        auto current_solution = problem.get_current_solution();
        auto obj = problem.objective(current_solution);
        auto obj_new = obj + 10;

        while (obj_new > obj) {
            obj = obj_new;
            current_solution = solve(problem, num_reruns, max_rectangle_in_subproblem, T, 0.80);
            obj_new = problem.objective(current_solution);
        }
        return current_solution;
    }

    std::vector<RectanglePlacement>
    solve(RectangleFittingProblem &problem,
          int num_reruns,
          int max_rectangle_in_subproblem,
          int T,
          double lock_threshold)
    {
        auto current_solution = problem.get_current_solution();
        if (current_solution.empty()) return current_solution;

        int current_obj = problem.objective(current_solution, T);

        int rerun_count = 0;
        int max_reruns = std::max(1, num_reruns);

        int current_max_subproblem = max_rectangle_in_subproblem;
        int max_global_limit = (int)current_solution.size();
        int growth_step = 5;

        while (rerun_count < max_reruns) {

            auto previous_solution = current_solution;
            int previous_obj = current_obj;

            auto [locked, active] = filter_by_utilization(
                current_solution, problem.get_box_length(), lock_threshold);

            if (active.empty()) break;

            auto optimized_active = solve_subproblem(
                active,
                problem.get_box_length(),
                current_max_subproblem,
                T
            );

            current_solution = merge_solutions(locked, optimized_active);
            problem.set_current_solution(current_solution);

            current_obj = problem.objective(current_solution, T);

            if (current_obj <= previous_obj) {
                current_max_subproblem = std::min(
                    current_max_subproblem + growth_step,
                    max_global_limit
                );
            }

            rerun_count++;
        }

        return current_solution;
    }

    std::vector<RectanglePlacement>
    solve_with_reruns(RectangleFittingProblem &problem,
                      int num_reruns,
                      int max_rectangle_in_subproblem,
                      int T) override
    {
        return solve(problem, num_reruns, max_rectangle_in_subproblem, T);
    }

    void reset_move_ids(std::vector<RectanglePlacement>& sol) {
        for (auto& r : sol)
            r.move_id = -1;
    }

protected:
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) = 0;

    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors_with_metadata(
        RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) {
        auto neighbors = construct_neighbors(problem, T);
        metadata.resize(neighbors.size());
        return neighbors;
    }

    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    filter_by_utilization(const std::vector<RectanglePlacement>& solution,
                         int box_length,
                         double lock_threshold)
    {
        std::unordered_map<int, long long> box_area;
        long long box_capacity = (long long)box_length * box_length;

        for (const auto& r : solution) {
            box_area[r.box_id] += (long long)r.width * r.height;
        }

        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;
        locked.reserve(solution.size());
        active.reserve(solution.size());

        for (const auto& r : solution) {
            double util = (double)box_area[r.box_id] / box_capacity;
            if (util >= lock_threshold) {
                locked.push_back(r);
            } else {
                active.push_back(r);
            }
        }

        return {locked, active};
    }

    std::vector<RectanglePlacement>
    solve_subproblem(const std::vector<RectanglePlacement>& rectangles,
                     int box_length,
                     int max_rectangle_in_subproblem,
                     int T)
    {
        if (rectangles.empty()) return {};

        std::unordered_map<int, std::vector<RectanglePlacement>> box_groups;
        for (const auto& r : rectangles) {
            box_groups[r.box_id].push_back(r);
        }

        std::vector<std::vector<RectanglePlacement>> batches;
        std::vector<RectanglePlacement> current_batch;
        for (auto& [box_id, group] : box_groups) {
            if (!current_batch.empty() &&
                (current_batch.size() + group.size() > max_rectangle_in_subproblem)) {
                batches.push_back(std::move(current_batch));
                current_batch = {};
            }
            current_batch.insert(current_batch.end(), group.begin(), group.end());
        }
        if (!current_batch.empty()) batches.push_back(std::move(current_batch));

        std::vector<std::future<std::vector<RectanglePlacement>>> futures;
        for (auto& batch : batches) {
            futures.push_back(std::async(std::launch::async, [this, batch, box_length, T]() {
                std::thread::id tid = std::this_thread::get_id();


                RectangleFittingProblem sub(box_length, batch);
                int base_iterations = std::min(500, (int)batch.size() * 10);
                return apply_local_search(sub, batch, base_iterations, 5, T);
            }));
        }

        std::vector<RectanglePlacement> merged;
        merged.reserve(rectangles.size());
        for (auto& f : futures) {
            auto res = f.get();
            merged.insert(merged.end(), res.begin(), res.end());
        }

        return merged;
    }

    std::vector<RectanglePlacement>
    merge_solutions(const std::vector<RectanglePlacement>& locked,
                   std::vector<RectanglePlacement> active)
    {
        int max_locked_box_id = -1;
        for (const auto& r : locked) {
            max_locked_box_id = std::max(max_locked_box_id, r.box_id);
        }

        std::unordered_map<int, int> box_id_mapping;
        int next_box_id = max_locked_box_id + 1;

        for (auto& r : active) {
            if (box_id_mapping.find(r.box_id) == box_id_mapping.end()) {
                box_id_mapping[r.box_id] = next_box_id++;
            }
            r.box_id = box_id_mapping[r.box_id];
        }

        std::vector<RectanglePlacement> merged = locked;
        merged.insert(merged.end(), active.begin(), active.end());

        reset_move_ids(merged);
        return merged;
    }

    std::vector<RectanglePlacement> apply_local_search(
        RectangleFittingProblem &problem,
        std::vector<RectanglePlacement> initial,
        int max_iterations = 2000,
        int max_non_improving = 5,
        int T = 1000)
    {
        auto current_solution = initial;
        int current_obj = problem.objective(current_solution, T);

        std::vector<RectanglePlacement> best_sol = current_solution;
        int best_obj = current_obj;

        int non_improving_count = 0;
        int iteration = 0;

        double start_temperature = T;
        double current_temperature = start_temperature;
        double min_temperature = 0.0;
        double cooling_rate = 0.94;

        while (non_improving_count < max_non_improving) {
            iteration++;
            current_temperature = start_temperature * std::pow(cooling_rate, iteration);
            current_temperature = std::max(min_temperature, current_temperature);
            int int_temperature = (int)current_temperature;

            std::vector<NeighborMetadata> metadata;
            auto neighbors = construct_neighbors_with_metadata(problem, int_temperature, metadata);
            if (neighbors.empty()) break;

            int best_neighbor_obj = current_obj;
            std::vector<RectanglePlacement> best_neighbor = current_solution;
            bool found_better_neighbor = false;

            for (size_t i = 0; i < neighbors.size(); ++i) {
                int obj;
                if (i < metadata.size() && metadata[i].can_use_delta) {
                    obj = problem.objective_delta(neighbors[i], metadata[i].changed_rect_idx, int_temperature);
                } else {
                    obj = problem.objective(neighbors[i], int_temperature);
                }

                if (obj > best_neighbor_obj) {
                    best_neighbor_obj = obj;
                    best_neighbor = neighbors[i];
                    found_better_neighbor = true;
                }
            }

            if (found_better_neighbor) {
                current_solution = best_neighbor;
                current_obj = best_neighbor_obj;

                if (current_obj > best_obj) {
                    best_sol = current_solution;
                    best_obj = current_obj;
                    non_improving_count = 0;
                } else {
                    non_improving_count++;
                }
                problem.set_current_solution(current_solution);
            } else {
                non_improving_count++;
            }
        }

        return best_sol;
    }
};

#endif
