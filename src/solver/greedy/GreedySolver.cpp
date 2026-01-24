#include "GreedySolver.h"
#include <algorithm>

std::vector<RectanglePlacement> GreedySolver::solve_one_step(RectangleFittingProblem &problem, int T) {
    auto original_solution = problem.get_current_solution();
    if (original_solution.empty()) return original_solution;

    // If this is the first step, initialize
    if (step_index == 0) {
        step_indices.clear();
        placed_rectangles.clear();
        occupancy_grids.clear();
        next_box_id = 0;
        
        // Create a list of indices sorted by the strategy
        step_indices.resize(original_solution.size());
        for (size_t i = 0; i < step_indices.size(); i++) {
            step_indices[i] = i;
        }
        
        // Sort indices based on strategy
        if (current_strategy == 0) { // Biggest first
            std::sort(step_indices.begin(), step_indices.end(),
                [&original_solution](int a, int b) {
                    int area_a = original_solution[a].width * original_solution[a].height;
                    int area_b = original_solution[b].width * original_solution[b].height;
                    return area_a > area_b;
                });
        } else if (current_strategy == 1) { // Smallest first
            std::sort(step_indices.begin(), step_indices.end(),
                [&original_solution](int a, int b) {
                    int area_a = original_solution[a].width * original_solution[a].height;
                    int area_b = original_solution[b].width * original_solution[b].height;
                    return area_a < area_b;
                });
        } else { // Best fit (biggest first for initial sort)
            std::sort(step_indices.begin(), step_indices.end(),
                [&original_solution](int a, int b) {
                    int area_a = original_solution[a].width * original_solution[a].height;
                    int area_b = original_solution[b].width * original_solution[b].height;
                    return area_a > area_b;
                });
        }
        
        // Initialize placed_rectangles to track placed status
        placed_rectangles.assign(original_solution.size(), false);
    }

    // If we've placed all rectangles, return current solution
    if (step_index >= step_indices.size()) {
        return problem.get_current_solution();
    }

    int original_idx = step_indices[step_index];
    const auto& rect_to_place = original_solution[original_idx];
    
    // Place the next rectangle
    RectanglePlacement placed;
    
    if (current_strategy == 2) { // Best fit strategy
        // For best fit, we need to consider all boxes for each rectangle
        RectanglePlacement best_placement = rect_to_place;
        int best_box_id = -1;
        int best_fit_score = -1;
        int L = problem.get_box_length();

        // Try all existing boxes
        for (int box_id = 0; box_id < next_box_id; box_id++) {
            if (box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            // Try both orientations
            for (int rot = 0; rot < 2; rot++) {
                if (rot == 1 && rect_to_place.width == rect_to_place.height) continue;

                int w = (rot == 0) ? rect_to_place.width : rect_to_place.height;
                int h = (rot == 0) ? rect_to_place.height : rect_to_place.width;

                if (w > L || h > L) continue;

                // Try all positions in this box
                for (int y = 0; y <= L - h; y++) {
                    for (int x = 0; x <= L - w; x++) {
                        if (!collides_with_occupancy(x, y, w, h, occupancy_grids[box_id], L)) {
                            int fit_score = calculate_fit_score(x, y, w, h, occupancy_grids[box_id], L);
                            if (fit_score > best_fit_score) {
                                best_fit_score = fit_score;
                                best_placement = RectanglePlacement(
                                    rect_to_place.width,
                                    rect_to_place.height,
                                    x, y, (rot == 1), box_id
                                );
                                best_box_id = box_id;
                            }
                        }
                    }
                }
            }
        }

        // If no good fit found, create new box
        if (best_box_id == -1) {
            placed = place_rectangle_step(problem, rect_to_place);
        } else {
            placed = best_placement;
            mark_occupied(placed, occupancy_grids, step_index, problem.get_box_length());
        }
    } else {
        // For biggest/smallest first, use simple placement
        placed = place_rectangle_step(problem, rect_to_place);
    }

    // Mark this rectangle as placed
    placed_rectangles[original_idx] = true;
    
    // Build the complete solution
    std::vector<RectanglePlacement> new_solution = original_solution;
    
    // Update the placed rectangle in the solution
    new_solution[original_idx] = placed;
    
    // For unplaced rectangles, keep them as-is (with default positions)
    for (size_t i = 0; i < new_solution.size(); i++) {
        if (!placed_rectangles[i]) {
            // Reset unplaced rectangles to default position
            new_solution[i].x = 0;
            new_solution[i].y = 0;
            new_solution[i].box_id = 0;
            new_solution[i].rotated = false;
        }
    }
    
    problem.set_current_solution(new_solution);
    step_index++;

    return new_solution;
}

// Step-by-step placement helper
RectanglePlacement GreedySolver::place_rectangle_step(RectangleFittingProblem &problem,
                                                     const RectanglePlacement &selected_rect) {
    int L = problem.get_box_length();
    int width = selected_rect.width;
    int height = selected_rect.height;

    for (int rot = 0; rot < 2; rot++) {
        if (rot == 1 && width == height) continue;
        int w = (rot == 0) ? width : height;
        int h = (rot == 0) ? height : width;
        if (w > L || h > L) continue;

        // Try existing boxes
        for (int box_id = 0; box_id < next_box_id; box_id++) {
            if (box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }
            const auto& grid = occupancy_grids[box_id];

            for (int y = 0; y <= L - h; y++) {
                for (int x = 0; x <= L - w; x++) {
                    if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                        RectanglePlacement placed = RectanglePlacement(width, height, x, y, (rot == 1), box_id);
                        mark_occupied(placed, occupancy_grids, step_index, L);
                        return placed;
                    }
                }
            }
        }

        // Create new box
        int box_id = next_box_id;
        if (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }
        if (!collides_with_occupancy(0, 0, w, h, occupancy_grids[box_id], L)) {
            RectanglePlacement placed = RectanglePlacement(width, height, 0, 0, (rot == 1), box_id);
            mark_occupied(placed, occupancy_grids, step_index, L);
            next_box_id++;
            return placed;
        }
    }

    // Fallback: create new box with potentially clipped rectangle
    int w = std::min(width, L);
    int h = std::min(height, L);
    bool rotated = false;
    if (w > L) w = L;
    if (h > L) h = L;

    int box_id = next_box_id;
    if (box_id >= occupancy_grids.size()) {
        occupancy_grids.push_back(std::vector<int>(L * L, -1));
    }
    RectanglePlacement placed = RectanglePlacement(width, height, 0, 0, rotated, box_id);
    mark_occupied(placed, occupancy_grids, step_index, L);
    next_box_id++;
    return placed;
}

std::vector<RectanglePlacement> GreedySolver::solve(RectangleFittingProblem &problem,
                                                   int num_reruns,
                                                   int max_rectangle_in_subproblem, int T) {
    // Reset step-by-step state when running complete solve
    reset_state();
    
    if (current_strategy == 0) {
        return solve_biggest_first(problem);
    } else if (current_strategy == 1) {
        return solve_smallest_first(problem);
    } else {
        return solve_best_fit(problem);
    }
}

std::vector<RectanglePlacement> GreedySolver::solve_biggest_first(RectangleFittingProblem &problem) {
    auto original_solution = problem.get_current_solution();
    if (original_solution.empty()) return original_solution;

    std::vector<RectanglePlacement> rectangles_to_place = original_solution;

    // Biggest first
    std::sort(rectangles_to_place.begin(), rectangles_to_place.end(),
        [](const RectanglePlacement& a, const RectanglePlacement& b) {
            int area_a = a.width * a.height;
            int area_b = b.width * b.height;
            return area_a > area_b;
        });

    // Place rectangles
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

std::vector<RectanglePlacement> GreedySolver::solve_smallest_first(RectangleFittingProblem &problem) {
    auto original_solution = problem.get_current_solution();
    if (original_solution.empty()) return original_solution;

    std::vector<RectanglePlacement> rectangles_to_place = original_solution;

    // Smallest first
    std::sort(rectangles_to_place.begin(), rectangles_to_place.end(),
        [](const RectanglePlacement& a, const RectanglePlacement& b) {
            int area_a = a.width * a.height;
            int area_b = b.width * b.height;
            return area_a < area_b;
        });

    // Place rectangles
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


RectanglePlacement GreedySolver::place_rectangle(RectangleFittingProblem &problem,
                                                const RectanglePlacement &selected_rect,
                                                std::vector<std::vector<int>>& occupancy_grids,
                                                int& next_box_id) {
    // Your existing implementation
    int L = problem.get_box_length();
    int width = selected_rect.width;
    int height = selected_rect.height;

    for (int rot = 0; rot < 2; rot++) {
        if (rot == 1 && width == height) continue;
        int w = (rot == 0) ? width : height;
        int h = (rot == 0) ? height : width;
        if (w > L || h > L) continue;

        for (int box_id = 0; box_id < next_box_id; box_id++) {
            if (box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }
            const auto& grid = occupancy_grids[box_id];

            for (int y = 0; y <= L - h; y++) {
                for (int x = 0; x <= L - w; x++) {
                    if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                        return RectanglePlacement(width, height, x, y, (rot == 1), box_id);
                    }
                }
            }
        }

        int box_id = next_box_id;
        if (box_id >= occupancy_grids.size()) {
            occupancy_grids.push_back(std::vector<int>(L * L, -1));
        }
        if (!collides_with_occupancy(0, 0, w, h, occupancy_grids[box_id], L)) {
            next_box_id++;
            return RectanglePlacement(width, height, 0, 0, (rot == 1), box_id);
        }
    }

    int w = std::min(width, L);
    int h = std::min(height, L);
    bool rotated = false;
    if (w > L) w = L;
    if (h > L) h = L;

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

std::vector<RectanglePlacement> GreedySolver::solve_best_fit(RectangleFittingProblem &problem) {
    auto original_solution = problem.get_current_solution();
    if (original_solution.empty()) return original_solution;

    // Sort by area descending (biggest first)
    std::vector<RectanglePlacement> rectangles_to_place = original_solution;
    std::sort(rectangles_to_place.begin(), rectangles_to_place.end(),
        [](const RectanglePlacement& a, const RectanglePlacement& b) {
            int area_a = a.width * a.height;
            int area_b = b.width * b.height;
            return area_a > area_b;
        });

    // Place rectangles using best-fit approach
    std::vector<RectanglePlacement> result;
    std::vector<std::vector<int>> occupancy_grids;
    int next_box_id = 0;
    int L = problem.get_box_length();

    for (size_t i = 0; i < rectangles_to_place.size(); i++) {
        RectanglePlacement best_placement = rectangles_to_place[i];
        int best_box_id = -1;
        int best_fit_score = -1;

        // Try all existing boxes to find best fit
        for (int box_id = 0; box_id < next_box_id; box_id++) {
            if (box_id >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            // Try both orientations
            for (int rot = 0; rot < 2; rot++) {
                if (rot == 1 && rectangles_to_place[i].width == rectangles_to_place[i].height) continue;

                int w = (rot == 0) ? rectangles_to_place[i].width : rectangles_to_place[i].height;
                int h = (rot == 0) ? rectangles_to_place[i].height : rectangles_to_place[i].width;

                if (w > L || h > L) continue;

                // Try all positions in this box
                for (int y = 0; y <= L - h; y++) {
                    for (int x = 0; x <= L - w; x++) {
                        if (!collides_with_occupancy(x, y, w, h, occupancy_grids[box_id], L)) {
                            // Calculate fit score (prefer positions that create less wasted space)
                            int fit_score = calculate_fit_score(x, y, w, h, occupancy_grids[box_id], L);

                            if (fit_score > best_fit_score) {
                                best_fit_score = fit_score;
                                best_placement = RectanglePlacement(
                                    rectangles_to_place[i].width,
                                    rectangles_to_place[i].height,
                                    x, y, (rot == 1), box_id
                                );
                                best_box_id = box_id;
                            }
                        }
                    }
                }
            }
        }

        // If no good fit found in existing boxes, create new box
        if (best_box_id == -1) {
            RectanglePlacement placed = place_rectangle(problem, rectangles_to_place[i], occupancy_grids, next_box_id);
            result.push_back(placed);
            mark_occupied(placed, occupancy_grids, i, L);
        } else {
            // Use the best placement found
            result.push_back(best_placement);
            mark_occupied(best_placement, occupancy_grids, i, L);
        }
    }

    return result;
}

int GreedySolver::calculate_fit_score(int x, int y, int w, int h, const std::vector<int>& grid, int L) {
    int score = 0;

    // Prefer placements that are adjacent to existing rectangles (creates compact packing)
    for (int py = std::max(0, y - 1); py <= std::min(L - 1, y + h); py++) {
        for (int px = std::max(0, x - 1); px <= std::min(L - 1, x + w); px++) {
            if ((px < x || px >= x + w || py < y || py >= y + h)) {
                if (grid[py * L + px] != -1) {
                    score += 10; // Bonus for adjacency
                }
            }
        }
    }

    // Prefer placements closer to corners (usually leads to better packing)
    int dist_to_corner = std::min(x, L - x - w) + std::min(y, L - y - h);
    score += (L * 2 - dist_to_corner); // Higher score for closer to corners

    return score;
}