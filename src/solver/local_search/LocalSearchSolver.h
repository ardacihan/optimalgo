// LocalSearchSolver.h
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

#include "../../solver/RectangleFittingProblemSolver.h"

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

        std::cout << "=== Starting Multi-Round Local Search ===" << std::endl;
        std::cout << "Initial objective: " << obj << std::endl;

        int round = 0;
        while (obj_new > obj) {
            round++;
            obj = obj_new;
            std::cout << "\n--- Round " << round << " ---" << std::endl;
            current_solution = solve(problem, 0, max_rectangle_in_subproblem, T, 0.80);
            obj_new = problem.objective(current_solution);
            std::cout << "Round " << round << " objective: " << obj_new << std::endl;
        }

        std::cout << "\n=== Local Search Complete ===" << std::endl;
        std::cout << "Final objective: " << obj << std::endl;
        std::cout << "Total rounds: " << round - 1 << std::endl;

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
        std::cout << "  Starting solve with lock_threshold=" << lock_threshold
                  << ", total rectangles=" << current_solution.size() << std::endl;

        int rerun_count = 0;
        int max_reruns = std::max(1, 1);

        int current_max_subproblem = max_rectangle_in_subproblem;
        int max_global_limit = (int)current_solution.size();
        int growth_step = 5;

        while (rerun_count < max_reruns) {
            rerun_count++;
            std::cout << "\n  Rerun " << rerun_count << "/" << max_reruns << std::endl;

            auto previous_solution = current_solution;
            int previous_obj = current_obj;

            auto [locked, active] = filter_by_utilization(
                current_solution, problem.get_box_length(), lock_threshold);

            std::cout << "    Locked rectangles: " << locked.size() << std::endl;
            std::cout << "    Active rectangles: " << active.size() << std::endl;

            if (active.empty()) {
                std::cout << "    No active rectangles to optimize" << std::endl;
                break;
            }

            auto optimized_active = solve_subproblem(
                active,
                problem.get_box_length(),
                current_max_subproblem,
                T
            );

            current_solution = merge_solutions(locked, optimized_active);
            problem.set_current_solution(current_solution);

            current_obj = problem.objective(current_solution, T);

            std::cout << "    Previous objective: " << previous_obj << std::endl;
            std::cout << "    Current objective: " << current_obj << std::endl;
            std::cout << "    Improvement: " << (current_obj - previous_obj) << std::endl;

            if (current_obj <= previous_obj) {
                current_max_subproblem = std::min(
                    current_max_subproblem + growth_step,
                    max_global_limit
                );
                std::cout << "    No improvement - increasing max subproblem size to "
                          << current_max_subproblem << std::endl;
            }
        }

        return current_solution;
    }


protected:
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) {
        auto neighbors = construct_neighbors(problem, T);
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

        std::cout << "      Created " << batches.size() << " subproblem batch(es)" << std::endl;
        for (size_t i = 0; i < batches.size(); i++) {
            // Collect unique box IDs in this batch
            std::set<int> box_ids;
            for (const auto& r : batches[i]) {
                box_ids.insert(r.box_id);
            }

            std::cout << "      Batch " << (i+1) << ": " << batches[i].size()
                      << " rectangles, box IDs: {";
            bool first = true;
            for (int box_id : box_ids) {
                if (!first) std::cout << ", ";
                std::cout << box_id;
                first = false;
            }
            std::cout << "}" << std::endl;
        }

        std::vector<std::future<std::pair<std::vector<RectanglePlacement>, int>>> futures;
        for (size_t i = 0; i < batches.size(); i++) {
            futures.push_back(std::async(std::launch::async, 
                [this, batch = batches[i], box_length, T, i]() {
                    std::thread::id tid = std::this_thread::get_id();

                    RectangleFittingProblem sub(box_length, batch);
                    int initial_obj = sub.objective(batch, T);
                    auto result = apply_local_search(sub, batch, 5, T);
                    int final_obj = sub.objective(result, T);
                    
                    std::cout << "        [Thread " << tid << "] Batch " << (i+1) 
                              << " - Initial obj: " << initial_obj 
                              << ", Final obj: " << final_obj 
                              << ", Improvement: " << (final_obj - initial_obj) << std::endl;
                    
                    return std::make_pair(result, final_obj);
            }));
        }

        std::vector<RectanglePlacement> merged;
        merged.reserve(rectangles.size());
        int total_subproblem_obj = 0;
        
        for (size_t i = 0; i < futures.size(); i++) {
            auto [res, obj] = futures[i].get();
            merged.insert(merged.end(), res.begin(), res.end());
            total_subproblem_obj += obj;
        }

        std::cout << "      Total subproblem objective: " << total_subproblem_obj << std::endl;

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

        return merged;
    }

    std::vector<RectanglePlacement> apply_local_search(
    RectangleFittingProblem &problem,
    std::vector<RectanglePlacement> initial,
    int max_non_improving = 2,
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

    std::cout << "          [LocalSearch] Starting - initial_obj=" << current_obj
              << ", max_non_improving=" << max_non_improving << std::endl;

    while (non_improving_count < max_non_improving) {
        iteration++;
        current_temperature = start_temperature * std::pow(cooling_rate, iteration);
        current_temperature = std::max(min_temperature, current_temperature);
        int int_temperature = (int)current_temperature;

        auto neighbors = construct_neighbors(problem, int_temperature);

        std::cout << "          [LocalSearch] Iter " << iteration
                  << ": T=" << int_temperature
                  << ", neighbors=" << neighbors.size()
                  << ", non_improving=" << non_improving_count << std::endl;

        if (neighbors.empty()) {
            std::cout << "          [LocalSearch] No neighbors found, stopping" << std::endl;
            break;
        }

        int best_neighbor_obj = current_obj;
        std::vector<RectanglePlacement> best_neighbor = current_solution;
        bool found_better_neighbor = false;

        for (size_t i = 0; i < neighbors.size(); ++i) {
            int obj = problem.objective(neighbors[i], int_temperature);
            if (obj > best_neighbor_obj) {
                best_neighbor_obj = obj;
                best_neighbor = neighbors[i];
                found_better_neighbor = true;
            }
        }

        if (found_better_neighbor) {
            current_solution = best_neighbor;
            current_obj = best_neighbor_obj;

            std::cout << "          [LocalSearch] Found improvement: " << current_obj
                      << " (+" << (current_obj - best_neighbor_obj + (best_neighbor_obj - problem.objective(best_sol, T))) << ")" << std::endl;

            if (current_obj > best_obj) {
                best_sol = current_solution;
                best_obj = current_obj;
                non_improving_count = 0;
                std::cout << "          [LocalSearch] NEW BEST: " << best_obj << std::endl;
            } else {
                non_improving_count++;
            }
            problem.set_current_solution(current_solution);
        } else {
            non_improving_count++;
            std::cout << "          [LocalSearch] No improvement this iteration" << std::endl;
        }
    }

    std::cout << "          [LocalSearch] Complete - best_obj=" << best_obj
              << ", iterations=" << iteration
              << ", improvement=" << (best_obj - problem.objective(initial, T)) << std::endl;

    return best_sol;
}
};

#endif