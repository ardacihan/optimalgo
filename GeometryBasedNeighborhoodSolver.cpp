//
// GeometryBasedNeighborhoodSolver.cpp
// Simplified version - direct collision checking without spatial hash/occupancy grid
//

#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <set>

// ============== Simple Box Data ==============

struct BoxData {
    std::vector<int> rect_indices;
    long long total_area;

    BoxData() : total_area(0) {}
};

// ============== Neighborhood Construction ==============

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    int MAX_NEIGHBORS = 60;

    // Build box data - SAME as original
    std::unordered_map<int, BoxData> box_data;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        box_data[box_id].rect_indices.push_back(i);
        box_data[box_id].total_area += solution[i].get_actual_width() * solution[i].get_actual_height();
    }

    // Identify frozen boxes (>90% utilization) - SAME as original
    std::unordered_set<int> frozen_boxes;
    for (const auto& [box_id, data] : box_data) {
        double utilization = (double)data.total_area / (double)(L * L);
        if (utilization > 0.9) frozen_boxes.insert(box_id);
    }

    // SIMPLIFIED: Direct collision check instead of occupancy grid
    auto collides_fast = [&](const RectanglePlacement& r, int target_box_id, int moved_idx) {
        auto it = box_data.find(target_box_id);
        if (it == box_data.end()) return false;

        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int other_idx : it->second.rect_indices) {
            if (other_idx == moved_idx) continue;
            const auto& other = solution[other_idx];

            int ow = other.get_actual_width();
            int oh = other.get_actual_height();

            // Check rectangle overlap
            bool overlap = !(r.x + w <= other.x ||
                           other.x + ow <= r.x ||
                           r.y + h <= other.y ||
                           other.y + oh <= r.y);

            if (overlap) return true;
        }
        return false;
    };

    // SAME as original
    auto is_valid_move = [&](const RectanglePlacement& moved, int target_box, int moved_idx) {
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L)
            return false;
        return !collides_fast(moved, target_box, moved_idx);
    };

    // SAME as original
    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    nbs.reserve(MAX_NEIGHBORS);

    // Find sparse boxes - SAME as original
    std::vector<std::pair<int, int>> sparse_boxes;
    for (const auto& [box_id, data] : box_data) {
        if (data.rect_indices.size() <= 3 || (data.total_area * 100) / (L * L) < 25)
            sparse_boxes.push_back({box_id, data.rect_indices.size()});
    }
    std::sort(sparse_boxes.begin(), sparse_boxes.end(),
              [](auto& a, auto& b) { return a.second < b.second; });

    // Generate neighbors - SAME logic as original
    for (const auto& [sparse_box_id, _] : sparse_boxes) {
        if (frozen_boxes.count(sparse_box_id)) continue;
        if (nbs.size() >= MAX_NEIGHBORS) break;
        const auto& sparse_data = box_data.at(sparse_box_id);

        for (int idx : sparse_data.rect_indices) {
            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            for (const auto& [target_box_id, target_data] : box_data) {
                if (frozen_boxes.count(target_box_id)) continue;
                if (target_box_id == sparse_box_id) continue;
                if (target_data.total_area + rect_area > (long long)L * L * 0.95) continue;

                std::set<std::pair<int,int>> contact_positions;
                int rect_w = rect.get_actual_width();
                int rect_h = rect.get_actual_height();

                // Corner positions - SAME as original
                contact_positions.insert({0, 0});
                contact_positions.insert({L - rect_w, 0});
                contact_positions.insert({0, L - rect_h});
                contact_positions.insert({L - rect_w, L - rect_h});

                // Positions next to existing rectangles - SAME as original
                int max_candidates = std::min(15, (int)target_data.rect_indices.size());
                for (int j = 0; j < max_candidates; j++) {
                    int j_idx = target_data.rect_indices[j];
                    const auto& rect_j = solution[j_idx];
                    int rj_w = rect_j.get_actual_width();
                    int rj_h = rect_j.get_actual_height();
                    contact_positions.insert({rect_j.x + rj_w, rect_j.y});
                    contact_positions.insert({rect_j.x - rect_w, rect_j.y});
                    contact_positions.insert({rect_j.x, rect_j.y + rj_h});
                    contact_positions.insert({rect_j.x, rect_j.y - rect_h});
                }

                // Try each position with both orientations - SAME as original
                for (const auto& [nx, ny] : contact_positions) {
                    if (nbs.size() >= MAX_NEIGHBORS) break;

                    for (int rot = 0; rot < 2; ++rot) {
                        RectanglePlacement moved = rect;
                        moved.box_id = target_box_id;
                        moved.x = nx;
                        moved.y = ny;
                        if (rot == 1) moved.rotated = !moved.rotated;

                        if (is_valid_move(moved, target_box_id, idx)) {
                            auto nb = solution;
                            nb[idx] = moved;
                            if (!add_neighbor(nb)) break;
                        }
                    }
                }
                if (nbs.size() >= MAX_NEIGHBORS) break;
            }
            if (nbs.size() >= MAX_NEIGHBORS) break;
        }
    }

    std::cout << "Generated " << nbs.size() << " valid neighbors (including rotations)" << std::endl;
    return nbs;
}