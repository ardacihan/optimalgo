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

// Structure to track neighbor generation metadata
struct NeighborMetadata {
    int changed_rect_idx;  // Which rectangle changed
    bool can_use_delta;    // Whether delta calculation is applicable
    
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
            current_solution = solve(problem, num_reruns, max_rectangle_in_subproblem, T, 0.85);
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
        std::cout << "\n=== STARTING ITERATIVE RERUN PROCESS ===" << std::endl;
        std::cout << "Total rectangles: " << current_solution.size() << std::endl;
        std::cout << "Initial objective: " << current_obj << std::endl;

        bool improved = true;
        int rerun_count = 0;
        int max_reruns = std::max(1, num_reruns);

        while (rerun_count < max_reruns) {
            std::cout << "\n=== RERUN " << rerun_count << " ===" << std::endl;

            auto previous_solution = current_solution;
            int previous_obj = current_obj;

            auto [locked, active] = filter_by_utilization(
                current_solution, problem.get_box_length(), lock_threshold);

            if (active.empty()) {
                std::cout << "No active rectangles remaining - stopping reruns" << std::endl;
                break;
            }

            auto optimized_active = solve_subproblem(
                active, problem.get_box_length(),
                max_rectangle_in_subproblem, T);

            current_solution = merge_solutions(locked, optimized_active);
            problem.set_current_solution(current_solution);

            current_obj = problem.objective(current_solution, T);
            std::cout << "Objective after rerun " << rerun_count << ": " << current_obj
                      << " (previous: " << previous_obj << ")" << std::endl;

            if (current_obj > previous_obj) {
                std::cout << "✓ Improved by " << (current_obj - previous_obj) << std::endl;
                improved = true;
            } else {
                std::cout << "✗ No improvement - stopping reruns" << std::endl;
                improved = false;
            }

            rerun_count++;
        }

        std::cout << "\n=== RERUN PROCESS COMPLETE ===" << std::endl;
        std::cout << "Total reruns performed: " << rerun_count << std::endl;
        std::cout << "Final objective: " << current_obj << std::endl;

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

    std::vector<RectanglePlacement> solve_one_step(RectangleFittingProblem &problem, int T = 1000) {
        std::vector<RectanglePlacement> solution = problem.get_current_solution();
        
        std::vector<NeighborMetadata> metadata;
        auto neighbors = construct_neighbors_with_metadata(problem, T, metadata);

        if (neighbors.empty()) return solution;

        int current_obj = problem.objective(solution, T);
        std::vector<RectanglePlacement> best_neighbor = solution;
        int best_obj = current_obj;

        for (size_t i = 0; i < neighbors.size(); ++i) {
            int obj;
            
            // Use delta calculation if possible
            if (i < metadata.size() && metadata[i].can_use_delta) {
                obj = problem.objective_delta(neighbors[i], metadata[i].changed_rect_idx, T);
            } else {
                obj = problem.objective(neighbors[i], T);
            }
            
            if (obj > best_obj) {
                best_obj = obj;
                best_neighbor = neighbors[i];
            }
        }

        if (best_obj > current_obj) {
            problem.set_current_solution(best_neighbor);
            return best_neighbor;
        }
        return solution;
    }

protected:
    // OLD: construct_neighbors without metadata (for backward compatibility)
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors(
        RectangleFittingProblem &problem, int T) = 0;
    
    // NEW: construct_neighbors with metadata for delta calculations
    virtual std::vector<std::vector<RectanglePlacement>> construct_neighbors_with_metadata(
        RectangleFittingProblem &problem, int T, std::vector<NeighborMetadata>& metadata) {
        // Default: call old method and mark all as non-delta
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
        std::unordered_map<int, int> box_rect_count;
        long long box_capacity = (long long)box_length * box_length;

        for (const auto& r : solution) {
            box_area[r.box_id] += (long long)r.width * r.height;
            box_rect_count[r.box_id]++;
        }

        std::cout << "\nBox utilization analysis:" << std::endl;
        std::set<int> locked_box_ids;
        std::set<int> active_box_ids;

        for (const auto& [box_id, area] : box_area) {
            double util = (double)area / box_capacity;
            bool is_locked = (util >= lock_threshold);

            std::cout << "  Box " << box_id << ": " << box_rect_count[box_id]
                      << " rectangles, " << (util * 100.0) << "% utilization"
                      << (is_locked ? " [LOCKED]" : " [ACTIVE]") << std::endl;

            if (is_locked) {
                locked_box_ids.insert(box_id);
            } else {
                active_box_ids.insert(box_id);
            }
        }

        std::vector<RectanglePlacement> locked;
        std::vector<RectanglePlacement> active;

        for (const auto& r : solution) {
            double util = (double)box_area[r.box_id] / box_capacity;
            if (util >= lock_threshold) {
                locked.push_back(r);
            } else {
                active.push_back(r);
            }
        }

        std::cout << "\nFiltering results:" << std::endl;
        std::cout << "  Locked: " << locked.size() << " rectangles in "
                  << locked_box_ids.size() << " boxes" << std::endl;
        std::cout << "  Active: " << active.size() << " rectangles in "
                  << active_box_ids.size() << " boxes" << std::endl;

        return {locked, active};
    }

    std::vector<RectanglePlacement>
    solve_subproblem(const std::vector<RectanglePlacement>& rectangles,
                     int box_length,
                     int max_rectangle_in_subproblem,
                     int T)
    {
        if ((int)rectangles.size() <= max_rectangle_in_subproblem) {
            RectangleFittingProblem sub(box_length, rectangles);
            auto solution = sub.get_current_solution();
            int base_iterations = std::min(500, (int)rectangles.size() * 10);
            return apply_local_search(sub, solution, base_iterations, 5, T);
        }

        auto [group1, group2] = split_rectangles(rectangles);

        // ✅ PARALLEL EXECUTION
        auto future1 = std::async(std::launch::async, [&] {
            return solve_subproblem(group1, box_length, max_rectangle_in_subproblem, T);
        });

        auto solution2 = solve_subproblem(group2, box_length, max_rectangle_in_subproblem, T);
        auto solution1 = future1.get();   // join

        std::vector<RectanglePlacement> merged = solution1;
        merged.insert(merged.end(), solution2.begin(), solution2.end());
        return merged;
    }


    std::pair<std::vector<RectanglePlacement>, std::vector<RectanglePlacement>>
    split_rectangles(const std::vector<RectanglePlacement>& rectangles)
    {
        std::unordered_map<int, std::vector<RectanglePlacement>> box_groups;
        for (const auto& r : rectangles) {
            box_groups[r.box_id].push_back(r);
        }

        if (box_groups.size() > 1) {
            std::vector<int> box_ids;
            for (const auto& [box_id, _] : box_groups) {
                box_ids.push_back(box_id);
            }
            std::sort(box_ids.begin(), box_ids.end());

            int mid = box_ids.size() / 2;
            std::vector<RectanglePlacement> group1, group2;

            for (int i = 0; i < (int)box_ids.size(); ++i) {
                if (i < mid) {
                    group1.insert(group1.end(),
                                 box_groups[box_ids[i]].begin(),
                                 box_groups[box_ids[i]].end());
                } else {
                    group2.insert(group2.end(),
                                 box_groups[box_ids[i]].begin(),
                                 box_groups[box_ids[i]].end());
                }
            }

            return {group1, group2};
        }

        std::vector<RectanglePlacement> sorted = rectangles;
        std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
            return (a.width * a.height) > (b.width * b.height);
        });

        int mid = sorted.size() / 2;
        std::vector<RectanglePlacement> group1(sorted.begin(), sorted.begin() + mid);
        std::vector<RectanglePlacement> group2(sorted.begin() + mid, sorted.end());

        return {group1, group2};
    }

    std::vector<RectanglePlacement>
    merge_solutions(const std::vector<RectanglePlacement>& locked,
                   std::vector<RectanglePlacement> active)
    {
        std::cout << "\nMerging solutions..." << std::endl;

        int max_locked_box_id = -1;
        for (const auto& r : locked) {
            max_locked_box_id = std::max(max_locked_box_id, r.box_id);
        }

        std::cout << "  Max locked box ID: " << max_locked_box_id << std::endl;

        std::unordered_map<int, int> box_id_mapping;
        int next_box_id = max_locked_box_id + 1;

        for (auto& r : active) {
            if (box_id_mapping.find(r.box_id) == box_id_mapping.end()) {
                box_id_mapping[r.box_id] = next_box_id++;
            }
            r.box_id = box_id_mapping[r.box_id];
        }

        std::cout << "  Remapped " << box_id_mapping.size() << " active box IDs" << std::endl;

        std::vector<RectanglePlacement> merged = locked;
        merged.insert(merged.end(), active.begin(), active.end());

        reset_move_ids(merged);

        std::unordered_set<int> final_boxes;
        for (const auto& r : merged) {
            final_boxes.insert(r.box_id);
        }

        std::cout << "  Final solution: " << merged.size() << " rectangles in "
                  << final_boxes.size() << " boxes" << std::endl;

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

        std::cout << "    Starting local search (initial obj: " << current_obj << ")" << std::endl;

        double start_temperature = T * 2.0;
        double current_temperature = start_temperature;
        double min_temperature = 0.0;
        double cooling_rate = 0.96;

        std::uniform_real_distribution<> dist(0.0, 1.0);

        while (non_improving_count < max_non_improving) {
            iteration++;

            current_temperature = start_temperature * std::pow(cooling_rate, iteration);
            current_temperature = std::max(min_temperature, current_temperature);

            int int_temperature = (int)current_temperature;

            std::vector<NeighborMetadata> metadata;
            auto neighbors = construct_neighbors_with_metadata(problem, int_temperature, metadata);

            if (neighbors.empty()) {
                std::cout << "    No neighbors at iteration " << iteration << std::endl;
                break;
            }

            int best_neighbor_obj = current_obj;
            std::vector<RectanglePlacement> best_neighbor = current_solution;

            // Evaluate neighbors using delta when possible
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
                }
            }

            bool accepted = false;

            if (best_neighbor_obj > current_obj) {
                accepted = true;
            }

            if (accepted) {
                current_solution = best_neighbor;
                current_obj = best_neighbor_obj;

                if (current_obj > best_obj) {
                    best_sol = current_solution;
                    best_obj = current_obj;
                    non_improving_count = 0;
                    std::cout << "    Iteration " << iteration << " (T=" << int_temperature
                              << "): new best = " << best_obj << std::endl;
                } else {
                    non_improving_count++;
                }

                problem.set_current_solution(current_solution);
            } else {
                non_improving_count++;
            }
        }

        int improvement = best_obj - problem.objective(initial, T);
        std::cout << "    Local search finished after " << iteration
                  << " iterations (best: " << best_obj
                  << ", improvement: " << improvement
                  << ", final T: " << (int)current_temperature << ")" << std::endl;

        return best_sol;
    }
};

#endif