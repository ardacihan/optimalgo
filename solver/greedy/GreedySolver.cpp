//
// Created by arda on 11/24/25.
//

#include "GreedySolver.h"
#include <algorithm>


RectanglePlacement BiggestFirstSelectionStrategy::select_rectangle(RectangleFittingProblem &problem) {
    auto solution = problem.get_current_solution();

    std::unordered_map<int, std::vector<const RectanglePlacement*>> boxes;
    for (const auto& rect : solution) {
        boxes[rect.box_id].push_back(&rect);
    }

    std::vector<const RectanglePlacement*> candidate_rects;

    for (const auto& rect : solution) {
        auto box_it = boxes.find(rect.box_id);
        if (box_it != boxes.end() && box_it->second.size() == 1) {
            // This rectangle is the only one in its box
            candidate_rects.push_back(&rect);
        }
    }

    if (candidate_rects.empty()) {
        for (const auto& rect : solution) {
            candidate_rects.push_back(&rect);
        }
    }

    const RectanglePlacement* largest = candidate_rects[0];
    int max_area = largest->get_actual_width() * largest->get_actual_height();

    for (size_t i = 1; i < candidate_rects.size(); i++) {
        int area = candidate_rects[i]->get_actual_width() * candidate_rects[i]->get_actual_height();
        if (area > max_area) {
            max_area = area;
            largest = candidate_rects[i];
        }
    }

    return *largest;
}

double MaximizeContactSelectionStrategy::calculate_simple_score(const RectanglePlacement& rect, bool is_alone) {
    int area = rect.get_actual_width() * rect.get_actual_height();
    double aspect = (double)std::max(rect.width, rect.height) /
                   std::min(rect.width, rect.height);

    // Base score on area (larger is better)
    double score = std::sqrt(area);

    // Bonus for being alone in box (we want to fill these boxes)
    if (is_alone) score *= 1.5;

    // Penalty for extreme aspect ratios
    score /= (1.0 + (aspect - 1.0) * 0.1);

    return score;
}

RectanglePlacement MaximizeContactSelectionStrategy::select_rectangle(RectangleFittingProblem &problem) {
    auto solution = problem.get_current_solution();

    // Find rectangles that are alone in their boxes
    std::unordered_map<int, int> box_counts;
    for (const auto& rect : solution) {
        box_counts[rect.box_id]++;
    }

    const RectanglePlacement* best_fit = &solution[0];
    double best_score = -1;

    for (const auto& rect : solution) {
        // Check if rectangle is alone in its box
        bool is_alone = (box_counts[rect.box_id] == 1);

        double score = calculate_simple_score(rect, is_alone);
        if (score > best_score) {
            best_score = score;
            best_fit = &rect;
        }
    }

    return *best_fit;
}

void GreedySolver::place_rectangle(RectangleFittingProblem &problem, RectanglePlacement &rectangle) {
    // Simple implementation using the existing placement logic
    std::vector<std::vector<int>> occupancy_grids;
    int next_box_id = 0;
    int L = problem.get_box_length();

    RectanglePlacement placed = place_rectangle(problem, rectangle, occupancy_grids, next_box_id);
    rectangle = placed;
}

RectanglePlacement GreedySolver::place_rectangle(RectangleFittingProblem &problem,
                                                const RectanglePlacement &selected_rect,
                                                std::vector<std::vector<int>>& occupancy_grids,
                                                int& next_box_id) {
    int L = problem.get_box_length();
    int width = selected_rect.width;
    int height = selected_rect.height;

    // Try both orientations
    for (int rot = 0; rot < 2; rot++) {
        if (rot == 1 && width == height) continue; // Skip rotation if square

        int w = (rot == 0) ? width : height;
        int h = (rot == 0) ? height : width;

        if (w > L || h > L) continue; // Skip if doesn't fit in box

        // Try existing boxes first
        for (int box_id = 0; box_id < next_box_id; box_id++) {
            // Ensure occupancy grid exists for this box
            if (box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            const auto& grid = occupancy_grids[box_id];

            // Try bottom-left positions
            for (int y = 0; y <= L - h; y++) {
                for (int x = 0; x <= L - w; x++) {
                    if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                        // Found valid position
                        return RectanglePlacement(width, height, x, y, (rot == 1), box_id);
                    }
                }
            }
        }

        // Try new box
        int box_id = next_box_id;
        if (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }

        // In new box, we can always place at (0,0)
        if (!collides_with_occupancy(0, 0, w, h, occupancy_grids[box_id], L)) {
            next_box_id++; // Increment for next placement
            return RectanglePlacement(width, height, 0, 0, (rot == 1), box_id);
        }
    }

    // If no placement found with rotation, try forcing into smallest possible dimensions
    int w = std::min(width, L);
    int h = std::min(height, L);
    bool rotated = false;

    if (w > L) w = L;
    if (h > L) h = L;

    // Try existing boxes
    for (int box_id = 0; box_id < next_box_id; box_id++) {
        if (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }

        const auto& grid = occupancy_grids[box_id];

        for (int y = 0; y <= L - h; y++) {
            for (int x = 0; x <= L - w; x++) {
                if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                    return RectanglePlacement(width, height, x, y, rotated, box_id);
                }
            }
        }
    }

    // Place in new box at (0,0)
    int box_id = next_box_id;
    if (box_id >= occupancy_grids.size()) {
        occupancy_grids.push_back(std::vector<int>(L * L, -1));
    }
    next_box_id++;
    return RectanglePlacement(width, height, 0, 0, rotated, box_id);
}

bool GreedySolver::collides_with_occupancy(int x, int y, int w, int h,
                                          const std::vector<int>& grid, int L) {
    for (int py = y; py < y + h; py++) {
        for (int px = x; px < x + w; px++) {
            if (px >= L || py >= L || px < 0 || py < 0) return true;
            if (grid[py * L + px] != -1) return true;
        }
    }
    return false;
}

void GreedySolver::mark_occupied(const RectanglePlacement& placement,
                                std::vector<std::vector<int>>& occupancy_grids,
                                int rect_idx, int L) {
    int box_id = placement.box_id;
    int w = placement.get_actual_width();
    int h = placement.get_actual_height();

    // Ensure the grid exists
    while (box_id >= occupancy_grids.size()) {
        occupancy_grids.push_back(std::vector<int>(L * L, -1));
    }

    auto& grid = occupancy_grids[box_id];
    for (int py = placement.y; py < placement.y + h; py++) {
        for (int px = placement.x; px < placement.x + w; px++) {
            if (px < L && py < L && px >= 0 && py >= 0) {
                grid[py * L + px] = rect_idx;
            }
        }
    }
}

std::vector<RectanglePlacement> GreedySolver::solve(RectangleFittingProblem &problem,
                                                   int num_reruns,
                                                   int max_rectangle_in_subproblem) {
    auto original_solution = problem.get_current_solution();
    if (original_solution.empty()) {
        return original_solution;
    }

    // Create a list of rectangles to place using selection strategy
    std::vector<RectanglePlacement> rectangles_to_place;

    // Select rectangles one by one using the selection strategy
    auto temp_problem = problem; // Copy to avoid modifying original
    while (!temp_problem.get_current_solution().empty()) {
        RectanglePlacement selected = selection_strategy->select_rectangle(temp_problem);
        rectangles_to_place.push_back(selected);

        // Remove the selected rectangle from temporary problem
        auto current_sol = temp_problem.get_current_solution();
        auto it = std::find_if(current_sol.begin(), current_sol.end(),
                              [&](const RectanglePlacement& r) {
                                  return r.x == selected.x && r.y == selected.y &&
                                         r.box_id == selected.box_id &&
                                         r.width == selected.width &&
                                         r.height == selected.height;
                              });
        if (it != current_sol.end()) {
            current_sol.erase(it);
            temp_problem.set_current_solution(current_sol);
        } else {
            // If we can't find the exact rectangle, break to avoid infinite loop
            break;
        }
    }

    // Now place the rectangles using the placement strategy
    std::vector<RectanglePlacement> result;
    std::vector<std::vector<int>> occupancy_grids;
    int next_box_id = 0;
    int L = problem.get_box_length();

    for (size_t i = 0; i < rectangles_to_place.size(); i++) {
        RectanglePlacement placed = place_rectangle(problem, rectangles_to_place[i],
                                                   occupancy_grids, next_box_id);
        result.push_back(placed);
        mark_occupied(placed, occupancy_grids, i, L);
    }

    return result;
}

std::vector<RectanglePlacement> GreedySolver::solve_with_reruns(RectangleFittingProblem &problem,
                                                               int num_reruns,
                                                               int max_rectangle_in_subproblem) {
    return solve(problem, num_reruns, max_rectangle_in_subproblem);
}