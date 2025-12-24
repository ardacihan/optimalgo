#include "GeometryBasedNeighborhoodSolver.h"
#include <algorithm>
#include <numeric>

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T)
{
    std::vector<std::vector<RectanglePlacement>> neighbors;
    const auto& solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();

    if (n == 0) return neighbors;

    const int MAX_NEIGHBORS = 2000;
    neighbors.reserve(MAX_NEIGHBORS);

    // Build occupancy data structures
    auto occupancy = build_occupancy_data(solution, L);

    // Create placement checker lambda
    auto can_place = [&](int rect_idx, int new_x, int new_y, bool new_rotated, int target_box) -> bool {
        return check_placement(solution[rect_idx], new_x, new_y, new_rotated,
                              target_box, occupancy.grids.at(target_box), L);
    };

    // Sort rectangles and boxes by priority
    auto rect_indices = get_sorted_rect_indices(solution, occupancy.counts);
    auto target_boxes = get_sorted_boxes(occupancy.boxes, occupancy.counts);

    // Add randomization if there are enough boxes
    if (target_boxes.size() > 3) {
        int offset = rand() % 3;
        std::rotate(target_boxes.begin(),
                   target_boxes.begin() + offset,
                   target_boxes.end());
    }

    // Generate cross-box placement neighbors
    generate_cross_box_neighbors(neighbors, solution, rect_indices, target_boxes,
                                can_place, MAX_NEIGHBORS);

    // Generate same-box shift neighbors
    generate_shift_neighbors(neighbors, solution, rect_indices, can_place,
                           L, MAX_NEIGHBORS);

    return neighbors;
}

GeometryBasedNeighborhoodSolver::OccupancyData
GeometryBasedNeighborhoodSolver::build_occupancy_data(
    const std::vector<RectanglePlacement>& solution, int L)
{
    OccupancyData data;

    for (const auto& rect : solution) {
        data.boxes.insert(rect.box_id);
    }

    for (int box_id : data.boxes) {
        std::vector<bool> grid(L * L, false);
        int count = 0;

        for (const auto& rect : solution) {
            if (rect.box_id != box_id) continue;

            int w = rect.get_actual_width();
            int h = rect.get_actual_height();

            for (int y = rect.y; y < rect.y + h && y < L; y++) {
                for (int x = rect.x; x < rect.x + w && x < L; x++) {
                    grid[y * L + x] = true;
                    count++;
                }
            }
        }

        data.grids[box_id] = std::move(grid);
        data.counts[box_id] = count;
    }

    return data;
}

bool GeometryBasedNeighborhoodSolver::check_placement(
    const RectanglePlacement& rect,
    int new_x, int new_y, bool new_rotated, int target_box,
    const std::vector<bool>& grid, int L) const
{
    int w = new_rotated ? rect.height : rect.width;
    int h = new_rotated ? rect.width : rect.height;

    if (new_x < 0 || new_y < 0 || new_x + w > L || new_y + h > L) {
        return false;
    }

    for (int dy = 0; dy < h; dy++) {
        for (int dx = 0; dx < w; dx++) {
            int gx = new_x + dx;
            int gy = new_y + dy;

            if (grid[gy * L + gx]) {
                // Allow overlap with original position in same box
                if (rect.box_id == target_box) {
                    int ow = rect.get_actual_width();
                    int oh = rect.get_actual_height();
                    bool in_original = gx >= rect.x && gx < rect.x + ow &&
                                      gy >= rect.y && gy < rect.y + oh;
                    if (!in_original) return false;
                } else {
                    return false;
                }
            }
        }
    }

    return true;
}

std::vector<int> GeometryBasedNeighborhoodSolver::get_sorted_rect_indices(
    const std::vector<RectanglePlacement>& solution,
    const std::unordered_map<int, int>& box_counts) const
{
    int n = solution.size();
    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        int occ_a = box_counts.at(solution[a].box_id);
        int occ_b = box_counts.at(solution[b].box_id);
        if (occ_a != occ_b) return occ_a < occ_b;

        int area_a = solution[a].width * solution[a].height;
        int area_b = solution[b].width * solution[b].height;
        return area_a > area_b;
    });

    return indices;
}

std::vector<int> GeometryBasedNeighborhoodSolver::get_sorted_boxes(
    const std::set<int>& used_boxes,
    const std::unordered_map<int, int>& box_counts) const
{
    std::vector<int> boxes(used_boxes.begin(), used_boxes.end());

    std::sort(boxes.begin(), boxes.end(), [&](int a, int b) {
        if (box_counts.at(a) != box_counts.at(b)) {
            return box_counts.at(a) > box_counts.at(b);
        }
        return a < b;
    });

    return boxes;
}

void GeometryBasedNeighborhoodSolver::generate_cross_box_neighbors(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    const std::vector<int>& rect_indices,
    const std::vector<int>& target_boxes,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int max_neighbors)
{
    int n = solution.size();

    for (int rect_idx : rect_indices) {
        if (neighbors.size() >= max_neighbors) break;

        const auto& moving_rect = solution[rect_idx];

        for (int target_box : target_boxes) {
            if (neighbors.size() >= max_neighbors) break;
            if (target_box == moving_rect.box_id) continue;

            // Try placing next to each rectangle in target box
            try_greedy_placements(neighbors, solution, rect_idx, target_box,
                                   can_place, max_neighbors);
        }
    }
}

bool GeometryBasedNeighborhoodSolver::try_adjacent_placements(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    int rect_idx, int target_box,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int max_neighbors)
{
    const auto& moving_rect = solution[rect_idx];
    int n = solution.size();

    for (int other_idx = 0; other_idx < n; other_idx++) {
        if (neighbors.size() >= max_neighbors) break;
        if (solution[other_idx].box_id != target_box) continue;

        const auto& anchor = solution[other_idx];

        // Try current orientation
        int w = moving_rect.get_actual_width();
        int h = moving_rect.get_actual_height();

        std::vector<std::pair<int, int>> positions = {
            {anchor.x - w, anchor.y},
            {anchor.x + anchor.get_actual_width(), anchor.y},
            {anchor.x, anchor.y - h},
            {anchor.x, anchor.y + anchor.get_actual_height()}
        };

        for (const auto& [new_x, new_y] : positions) {
            if (can_place(rect_idx, new_x, new_y, moving_rect.rotated, target_box)) {
                auto neighbor = solution;
                neighbor[rect_idx].box_id = target_box;
                neighbor[rect_idx].x = new_x;
                neighbor[rect_idx].y = new_y;
                neighbors.push_back(neighbor);
                break;
            }
        }

        // Try rotated orientation if rectangle is not square
        if (moving_rect.width != moving_rect.height) {
            int rot_w = moving_rect.height;
            int rot_h = moving_rect.width;

            std::vector<std::pair<int, int>> rot_positions = {
                {anchor.x - rot_w, anchor.y},
                {anchor.x + anchor.get_actual_width(), anchor.y},
                {anchor.x, anchor.y - rot_h},
                {anchor.x, anchor.y + anchor.get_actual_height()}
            };

            for (const auto& [new_x, new_y] : rot_positions) {
                if (can_place(rect_idx, new_x, new_y, !moving_rect.rotated, target_box)) {
                    auto neighbor = solution;
                    neighbor[rect_idx].box_id = target_box;
                    neighbor[rect_idx].x = new_x;
                    neighbor[rect_idx].y = new_y;
                    neighbor[rect_idx].rotated = !moving_rect.rotated;
                    neighbors.push_back(neighbor);
                    break;
                }
            }
        }
    }

    return false;
}

bool GeometryBasedNeighborhoodSolver::try_skyline_placements(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    int rect_idx, int target_box, int box_length,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int max_neighbors)
{
    const auto& moving_rect = solution[rect_idx];

    // Build skyline for target box
    std::vector<std::pair<int, int>> skyline;  // (x_position, height)
    skyline.push_back({0, 0});
    skyline.push_back({box_length, 0});

    for (const auto& rect : solution) {
        if (rect.box_id != target_box) continue;

        int x1 = rect.x;
        int x2 = rect.x + rect.get_actual_width();
        int y = rect.y + rect.get_actual_height();

        // Update skyline (simplified - you may want a more sophisticated merge)
        for (int x = x1; x < x2 && x < box_length; x++) {
            bool found = false;
            for (auto& [sx, sh] : skyline) {
                if (sx == x) {
                    sh = std::max(sh, y);
                    found = true;
                    break;
                }
            }
            if (!found) {
                skyline.push_back({x, y});
            }
        }
    }

    // Sort skyline by x position
    std::sort(skyline.begin(), skyline.end());

    // Try both orientations
    std::vector<bool> orientations = {moving_rect.rotated};
    if (moving_rect.width != moving_rect.height) {
        orientations.push_back(!moving_rect.rotated);
    }

    for (bool rotated : orientations) {
        if (neighbors.size() >= max_neighbors) break;

        int w = rotated ? moving_rect.height : moving_rect.width;
        int h = rotated ? moving_rect.width : moving_rect.height;

        // Try placing at each skyline position
        for (const auto& [x, y] : skyline) {
            if (x + w > box_length) continue;

            if (can_place(rect_idx, x, y, rotated, target_box)) {
                auto neighbor = solution;
                neighbor[rect_idx].box_id = target_box;
                neighbor[rect_idx].x = x;
                neighbor[rect_idx].y = y;
                neighbor[rect_idx].rotated = rotated;
                neighbors.push_back(neighbor);
                return true;
            }
        }
    }

    return false;
}

bool GeometryBasedNeighborhoodSolver::try_greedy_placements(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    int rect_idx, int target_box,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int max_neighbors)
{
    const auto& moving_rect = solution[rect_idx];
    int n = solution.size();

    // Collect all rectangles in target box
    std::vector<int> target_rect_indices;
    for (int i = 0; i < n; i++) {
        if (solution[i].box_id == target_box) {
            target_rect_indices.push_back(i);
        }
    }

    // Try both orientations
    std::vector<bool> orientations = {moving_rect.rotated};
    if (moving_rect.width != moving_rect.height) {
        orientations.push_back(!moving_rect.rotated);
    }

    for (bool rotated : orientations) {
        if (neighbors.size() >= max_neighbors) break;

        int w = rotated ? moving_rect.height : moving_rect.width;
        int h = rotated ? moving_rect.width : moving_rect.height;

        // Strategy 1: Try corner positions (greedy-style)
        std::vector<std::pair<int, int>> corner_positions = {
            {0, 0},  // Top-left corner
        };

        for (const auto& [x, y] : corner_positions) {
            if (neighbors.size() >= max_neighbors) break;
            if (can_place(rect_idx, x, y, rotated, target_box)) {
                auto neighbor = solution;
                neighbor[rect_idx].box_id = target_box;
                neighbor[rect_idx].x = x;
                neighbor[rect_idx].y = y;
                neighbor[rect_idx].rotated = rotated;
                neighbors.push_back(neighbor);
                return true;  // Found a placement
            }
        }

        // Strategy 2: Try bottom-left style positions (scan from left to right, bottom to top)
        std::vector<std::pair<int, int>> bl_positions;

        // Build a set of candidate positions based on existing rectangles
        for (int idx : target_rect_indices) {
            const auto& anchor = solution[idx];
            int aw = anchor.get_actual_width();
            int ah = anchor.get_actual_height();

            // Positions that align with existing rectangles (greedy-style)
            bl_positions.push_back({anchor.x + aw, anchor.y});  // Right of anchor
            bl_positions.push_back({anchor.x, anchor.y + ah});  // Above anchor
            bl_positions.push_back({0, anchor.y + ah});         // Left edge, above anchor
            bl_positions.push_back({anchor.x + aw, 0});         // Bottom edge, right of anchor
        }

        // Sort positions by bottom-left preference: prioritize lower y, then lower x
        std::sort(bl_positions.begin(), bl_positions.end(),
                 [](const auto& a, const auto& b) {
                     if (a.second != b.second) return a.second < b.second;
                     return a.first < b.first;
                 });

        // Try positions in order
        for (const auto& [x, y] : bl_positions) {
            if (neighbors.size() >= max_neighbors) break;
            if (can_place(rect_idx, x, y, rotated, target_box)) {
                auto neighbor = solution;
                neighbor[rect_idx].box_id = target_box;
                neighbor[rect_idx].x = x;
                neighbor[rect_idx].y = y;
                neighbor[rect_idx].rotated = rotated;
                neighbors.push_back(neighbor);
                return true;  // Found a placement
            }
        }

        // Strategy 3: Try all 4 sides of each existing rectangle (more exhaustive)
        for (int idx : target_rect_indices) {
            if (neighbors.size() >= max_neighbors) break;

            const auto& anchor = solution[idx];
            int aw = anchor.get_actual_width();
            int ah = anchor.get_actual_height();

            std::vector<std::pair<int, int>> adjacent_positions = {
                {anchor.x - w, anchor.y},      // Left
                {anchor.x + aw, anchor.y},     // Right
                {anchor.x, anchor.y - h},      // Below
                {anchor.x, anchor.y + ah}      // Above
            };

            for (const auto& [x, y] : adjacent_positions) {
                if (can_place(rect_idx, x, y, rotated, target_box)) {
                    auto neighbor = solution;
                    neighbor[rect_idx].box_id = target_box;
                    neighbor[rect_idx].x = x;
                    neighbor[rect_idx].y = y;
                    neighbor[rect_idx].rotated = rotated;
                    neighbors.push_back(neighbor);
                    return true;  // Found a placement
                }
            }
        }
    }

    return false;  // No valid placement found
}

void GeometryBasedNeighborhoodSolver::generate_shift_neighbors(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    const std::vector<int>& rect_indices,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int L, int max_neighbors)
{
    for (int rect_idx : rect_indices) {
        if (neighbors.size() >= max_neighbors) break;

        // Try shifting in each direction: left, right, up, down
        add_shift_neighbor(neighbors, solution, rect_idx, -1, 0, can_place, L);
        add_shift_neighbor(neighbors, solution, rect_idx, 1, 0, can_place, L);
        add_shift_neighbor(neighbors, solution, rect_idx, 0, -1, can_place, L);
        add_shift_neighbor(neighbors, solution, rect_idx, 0, 1, can_place, L);
    }
}

void GeometryBasedNeighborhoodSolver::add_shift_neighbor(
    std::vector<std::vector<RectanglePlacement>>& neighbors,
    const std::vector<RectanglePlacement>& solution,
    int rect_idx, int dx, int dy,
    const std::function<bool(int, int, int, bool, int)>& can_place,
    int L)
{
    const auto& rect = solution[rect_idx];
    int box_id = rect.box_id;
    int w = rect.get_actual_width();
    int h = rect.get_actual_height();

    // Calculate maximum shift in the given direction
    int limit;
    if (dx < 0) {
        limit = rect.x;
    } else if (dx > 0) {
        limit = L - (rect.x + w);
    } else if (dy < 0) {
        limit = rect.y;
    } else {
        limit = L - (rect.y + h);
    }

    int max_shift = 0;
    for (int shift = 1; shift <= limit; shift++) {
        int new_x = rect.x + dx * shift;
        int new_y = rect.y + dy * shift;

        if (!can_place(rect_idx, new_x, new_y, rect.rotated, box_id)) {
            break;
        }
        max_shift = shift;
    }

    if (max_shift > 0) {
        auto neighbor = solution;
        neighbor[rect_idx].x = rect.x + dx * max_shift;
        neighbor[rect_idx].y = rect.y + dy * max_shift;
        neighbors.push_back(neighbor);
    }
}

std::vector<RectanglePlacement>
GeometryBasedNeighborhoodSolver::solve_one_step(
    RectangleFittingProblem &problem, int T)
{
    auto neighbors = construct_neighbors(problem, T);
    int best_obj = problem.objective(problem.get_current_solution());
    std::vector<RectanglePlacement> best_solution = problem.get_current_solution();
    bool improved = false;

    for (const auto& neighbor : neighbors) {
        // Temporarily apply the neighbor solution
        std::vector<RectanglePlacement> original_solution = problem.get_current_solution();
        problem.set_current_solution(neighbor);

        // Calculate objective for this neighbor
        int neighbor_obj = problem.objective(neighbor);

        // Check if this neighbor is better
        if (neighbor_obj > best_obj) {
            best_obj = neighbor_obj;
            best_solution = neighbor;
            improved = true;
        }

        // Restore original solution to continue exploring neighbors
        problem.set_current_solution(original_solution);
    }

    // If we found an improvement, update the problem with the best solution
    if (improved) {
        problem.set_current_solution(best_solution);
    }

    return problem.get_current_solution();
}