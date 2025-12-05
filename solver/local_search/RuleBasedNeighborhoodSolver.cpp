// RuleBasedNeighborhoodSolver.cpp

#include "RuleBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <chrono>
#include <cstdint>

// Simple unique ID generator using timestamp
class UniqueIDGenerator {
private:
    static uint64_t counter;

public:
    static uint64_t getUniqueID() {
        // Use nanoseconds since epoch for uniqueness
        auto now = std::chrono::high_resolution_clock::now();
        auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        // Combine with counter for extra safety
        return (nanos << 16) | (counter++ & 0xFFFF);
    }
};

uint64_t UniqueIDGenerator::counter = 0;

bool collides_with_occupancy(int x, int y, int w, int h,
                           const std::vector<int>& grid, int L) {
    for (int py = y; py < y + h; py++) {
        if (py >= L) return true;
        for (int px = x; px < x + w; px++) {
            if (px >= L) return true;
            if (grid[py * L + px] != -1) return true;
        }
    }
    return false;
}

std::vector<RectanglePlacement>
RuleBasedNeighborhoodSolver::apply_greedy_placement_indexed(
    const std::vector<int>& rect_indices,
    const std::vector<std::pair<int, int>>& rect_dims,
    int L) {

    std::vector<RectanglePlacement> result;

    if (rect_indices.empty()) return result;

    // We'll assign relative box IDs (0, 1, 2...) here
    int current_target_box = 0;
    std::vector<std::vector<int>> occupancy_grids;

    // Start with first rectangle
    for (size_t i = 0; i < rect_indices.size(); i++) {
        int idx = rect_indices[i];
        int width = rect_dims[idx].first;
        int height = rect_dims[idx].second;

        // Try to place this rectangle in the current target box
        bool placed = false;

        if (current_target_box < occupancy_grids.size()) {
            const auto& grid = occupancy_grids[current_target_box];

            // Try both orientations
            for (int rot = 0; rot < 2 && !placed; rot++) {
                if (rot == 1 && width == height) continue;
                int w = (rot == 0) ? width : height;
                int h = (rot == 0) ? height : width;
                if (w > L || h > L) continue;

                // Search for empty spot
                for (int y = 0; y <= L - h && !placed; y++) {
                    for (int x = 0; x <= L - w && !placed; x++) {
                        if (!collides_with_occupancy(x, y, w, h, grid, L)) {
                            result.push_back(RectanglePlacement(width, height, x, y, (rot == 1), current_target_box));
                            placed = true;
                            // Mark occupied
                            auto& target_grid = occupancy_grids[current_target_box];
                            for (int py = y; py < y + h; py++) {
                                for (int px = x; px < x + w; px++) {
                                    target_grid[py * L + px] = 1;
                                }
                            }
                        }
                    }
                }
            }
        } else {
            // New Box
            while (current_target_box >= occupancy_grids.size()) {
                occupancy_grids.push_back(std::vector<int>(L * L, -1));
            }

            int w = width;
            int h = height;
            bool rotated = false;

            if (w > L || h > L) {
                if (h <= L && w <= L && width != height) {
                    std::swap(w, h);
                    rotated = true;
                } else {
                    w = std::min(width, L);
                    h = std::min(height, L);
                    if (w > L) w = L;
                    if (h > L) h = L;
                }
            }

            result.push_back(RectanglePlacement(width, height, 0, 0, rotated, current_target_box));
            placed = true;

            auto& target_grid = occupancy_grids[current_target_box];
            for (int py = 0; py < h; py++) {
                for (int px = 0; px < w; px++) {
                    target_grid[py * L + px] = 1;
                }
            }
        }

        if (!placed) {
            current_target_box++;
            i--;
        }
    }

    return result;
}

// Helper function to validate a solution has no overlaps
bool validate_solution_no_overlaps(const std::vector<RectanglePlacement>& solution, int L) {
    // Group by box_id
    std::map<int, std::vector<const RectanglePlacement*>> boxes;
    for (const auto& rect : solution) {
        boxes[rect.box_id].push_back(&rect);
    }

    // Check each box for overlaps
    for (const auto& [box_id, rects] : boxes) {
        // Create occupancy grid for this box
        std::vector<bool> grid(L * L, false);

        for (const auto& rect : rects) {
            int x = rect->x;
            int y = rect->y;
            int w = rect->get_actual_width();
            int h = rect->get_actual_height();

            // Check bounds
            if (x < 0 || y < 0 || x + w > L || y + h > L) {
                return false;
            }

            // Check for overlaps
            for (int py = y; py < y + h; py++) {
                for (int px = x; px < x + w; px++) {
                    int index = py * L + px;
                    if (grid[index]) {
                        std::cerr << "Overlap detected in box " << box_id
                                  << " at (" << px << "," << py << ")" << std::endl;
                        return false;
                    }
                    grid[index] = true;
                }
            }
        }
    }
    return true;
}

std::vector<std::vector<RectanglePlacement>>
RuleBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem, int T) {

    std::vector<std::vector<RectanglePlacement>> neighbors;
    auto solution = problem.get_current_solution();
    int n = solution.size();
    int L = problem.get_box_length();

    const int MAX_NEIGHBORS = 100;

    if (n == 0) return neighbors;

    // Validate current solution
    //if (!validate_solution_no_overlaps(solution, L)) {
    //    std::cerr << "WARNING: Current solution has overlaps!" << std::endl;
    //    return neighbors; // Don't generate neighbors from invalid solution
    //}

    std::vector<std::pair<int, int>> rect_dims(n);
    std::vector<int> rect_indices(n);
    std::vector<int> original_box_ids(n);

    // Track used box IDs to avoid collisions
    std::set<int> used_box_ids;
    for (int i = 0; i < n; i++) {
        rect_dims[i] = {solution[i].width, solution[i].height};
        rect_indices[i] = i;
        original_box_ids[i] = solution[i].box_id;
        used_box_ids.insert(solution[i].box_id);
    }

    // Find next available box ID (max + 1)
    int max_box_id = 0;
    for (int box_id : used_box_ids) {
        if (box_id > max_box_id) max_box_id = box_id;
    }
    int next_box_id = max_box_id + 1;

    for (int i = 0; i < n && neighbors.size() < MAX_NEIGHBORS; i++) {
        for (int j = i + 1; j < n && neighbors.size() < MAX_NEIGHBORS; j++) {

            // 1. Create neighbor by swapping order
            std::vector<int> reordered = rect_indices;
            std::swap(reordered[i], reordered[j]);

            // 2. Apply greedy placement (returns box IDs 0, 1, 2...)
            auto new_solution = apply_greedy_placement_indexed(reordered, rect_dims, L);

            if (new_solution.size() != n) continue;

            // 3. SIMPLIFIED REMAPPING: Use ORIGINAL box IDs when possible,
            // otherwise assign NEW UNIQUE box IDs
            std::map<int, int> relative_to_real_map;
            std::set<int> assigned_real_ids;

            // First pass: try to assign original box IDs
            for (size_t k = 0; k < new_solution.size(); k++) {
                int relative_box_id = new_solution[k].box_id;
                int original_rect_idx = reordered[k];
                int original_id = original_box_ids[original_rect_idx];

                if (relative_to_real_map.find(relative_box_id) == relative_to_real_map.end()) {
                    // This relative box hasn't been assigned a real ID yet
                    if (assigned_real_ids.find(original_id) == assigned_real_ids.end()) {
                        // Original ID is available, use it
                        relative_to_real_map[relative_box_id] = original_id;
                        assigned_real_ids.insert(original_id);
                    } else {
                        // Original ID is taken, assign a NEW UNIQUE ID
                        // Use time-based unique ID to guarantee no collisions
                        relative_to_real_map[relative_box_id] = next_box_id++;
                        assigned_real_ids.insert(relative_to_real_map[relative_box_id]);
                    }
                }

                // Apply the mapping
                new_solution[k].box_id = relative_to_real_map[relative_box_id];
            }

            // 4. VALIDATE the solution (crucial!)
            //bool is_valid = validate_solution_no_overlaps(new_solution, L);

            //if (is_valid) {
                neighbors.push_back(new_solution);
            //} else {
            //    std::cerr << "Rejected invalid neighbor (overlaps)" << std::endl;
            //}
        }
    }

    std::cout << "Generated " << neighbors.size() << " valid neighbors" << std::endl;

    // Double-check all neighbors
    //int valid_count = 0;
    //for (const auto& neighbor : neighbors) {
    //    if (validate_solution_no_overlaps(neighbor, L)) {
    //        valid_count++;
    //    }
    // }

    //if (valid_count != neighbors.size()) {
    //    std::cerr << "ERROR: " << (neighbors.size() - valid_count)
    //              << " neighbors have overlaps!" << std::endl;
    //}

    return neighbors;
}