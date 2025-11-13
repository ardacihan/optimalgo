#include "GeometryBasedNeighborhoodSolver.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <random>

std::vector<std::vector<RectanglePlacement>>
GeometryBasedNeighborhoodSolver::construct_neighbors(RectangleFittingProblem &problem) {

    std::vector<std::vector<RectanglePlacement>> neighbors;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return neighbors;

    const int MAX_NEIGHBORS = 400;
    std::random_device rd;
    std::mt19937 gen(rd());

    // Build box information - CRITICAL for sparse box detection
    std::unordered_map<int, std::vector<int>> box_rects;
    std::unordered_map<int, long long> box_areas;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        box_rects[box_id].push_back(i);
        box_areas[box_id] += solution[i].get_actual_width() * solution[i].get_actual_height();
    }

    // Identify sparse boxes (<25% utilization or very few rectangles)
    std::vector<int> sparse_boxes;
    for (const auto& [box_id, area] : box_areas) {
        double utilization = (double)area / (double)(L * L);
        if (utilization < 0.25 || box_rects[box_id].size() <= 2) {
            sparse_boxes.push_back(box_id);
        }
    }

    // Simple collision checker
    auto has_collision = [&](const RectanglePlacement& rect, int target_box, int skip_idx) {
        int w = rect.get_actual_width();
        int h = rect.get_actual_height();

        // Boundary check
        if (rect.x < 0 || rect.y < 0 || rect.x + w > L || rect.y + h > L) {
            return true;
        }

        // Check collisions in target box
        for (int other_idx : box_rects[target_box]) {
            if (other_idx == skip_idx) continue;

            const auto& other = solution[other_idx];
            int ow = other.get_actual_width();
            int oh = other.get_actual_height();

            if (rect.x < other.x + ow && rect.x + w > other.x &&
                rect.y < other.y + oh && rect.y + h > other.y) {
                return true;
            }
        }
        return false;
    };

    // Generate random positions
    auto generate_random_position = [&](int rect_width, int rect_height) -> std::pair<int, int> {
        std::uniform_int_distribution<> x_dist(0, L - rect_width);
        std::uniform_int_distribution<> y_dist(0, L - rect_height);
        return {x_dist(gen), y_dist(gen)};
    };

    // STRATEGY 1: Aggressive sparse box merging - prioritize this!
    if (!sparse_boxes.empty()) {
        //std::cout << "Found " << sparse_boxes.size() << " sparse boxes, generating merge neighbors..." << std::endl;

        // Try to move rectangles from sparse boxes to any non-sparse box
        for (int sparse_box : sparse_boxes) {
            if (neighbors.size() >= MAX_NEIGHBORS * 2 / 3) break; // Reserve most capacity for merging

            for (int rect_idx : box_rects[sparse_box]) {
                if (neighbors.size() >= MAX_NEIGHBORS * 2 / 3) break;

                const auto& rect = solution[rect_idx];
                int rect_area = rect.get_actual_width() * rect.get_actual_height();

                // Try all non-sparse boxes as targets
                for (const auto& [target_box, target_rects] : box_rects) {
                    if (neighbors.size() >= MAX_NEIGHBORS * 2 / 3) break;
                    if (target_box == sparse_box) continue;
                    if (std::find(sparse_boxes.begin(), sparse_boxes.end(), target_box) != sparse_boxes.end()) continue;

                    // Check capacity
                    if (box_areas[target_box] + rect_area > L * L * 0.95) continue;

                    // Try multiple positions in target box
                    for (int attempt = 0; attempt < 3; attempt++) {
                        if (neighbors.size() >= MAX_NEIGHBORS * 2 / 3) break;

                        for (int rot = 0; rot < 2; rot++) {
                            RectanglePlacement moved = rect;
                            moved.box_id = target_box;
                            moved.rotated = (rot == 1) ? !rect.rotated : rect.rotated;

                            auto [x, y] = generate_random_position(moved.get_actual_width(), moved.get_actual_height());
                            moved.x = x;
                            moved.y = y;

                            if (!has_collision(moved, target_box, rect_idx)) {
                                auto neighbor = solution;
                                neighbor[rect_idx] = moved;
                                neighbors.push_back(neighbor);
                                break; // One valid move per attempt
                            }
                        }
                    }
                }
            }
        }
    }

    // STRATEGY 2: Standard geometric operations (only use remaining capacity)

    // Operation 1: Move rectangle to random position in another box
    for (int attempt = 0; attempt < MAX_NEIGHBORS / 6; attempt++) {
        if (neighbors.size() >= MAX_NEIGHBORS) break;

        int rect_idx = std::uniform_int_distribution<>(0, n - 1)(gen);
        const auto& rect = solution[rect_idx];
        int current_box = rect.box_id;

        // Pick random target box (different from current)
        if (box_rects.size() <= 1) continue;

        int target_box = current_box;
        int attempts = 0;
        while (target_box == current_box && attempts < 10) {
            auto it = box_rects.begin();
            std::advance(it, std::uniform_int_distribution<>(0, box_rects.size() - 1)(gen));
            target_box = it->first;
            attempts++;
        }
        if (target_box == current_box) continue;

        // Try both orientations
        for (int rot = 0; rot < 2; rot++) {
            RectanglePlacement moved = rect;
            moved.box_id = target_box;
            moved.rotated = (rot == 1) ? !rect.rotated : rect.rotated;

            auto [x, y] = generate_random_position(moved.get_actual_width(), moved.get_actual_height());
            moved.x = x;
            moved.y = y;

            if (!has_collision(moved, target_box, rect_idx)) {
                auto neighbor = solution;
                neighbor[rect_idx] = moved;
                neighbors.push_back(neighbor);
                break;
            }
        }
    }

    // Operation 2: Move rectangle to random position in same box
    for (int attempt = 0; attempt < MAX_NEIGHBORS / 6; attempt++) {
        if (neighbors.size() >= MAX_NEIGHBORS) break;

        int rect_idx = std::uniform_int_distribution<>(0, n - 1)(gen);
        const auto& rect = solution[rect_idx];
        int current_box = rect.box_id;

        RectanglePlacement moved = rect;
        auto [x, y] = generate_random_position(moved.get_actual_width(), moved.get_actual_height());
        moved.x = x;
        moved.y = y;

        if (!has_collision(moved, current_box, rect_idx)) {
            auto neighbor = solution;
            neighbor[rect_idx] = moved;
            neighbors.push_back(neighbor);
        }
    }

    // Operation 3: Rotate rectangle in place
    for (int attempt = 0; attempt < MAX_NEIGHBORS / 6; attempt++) {
        if (neighbors.size() >= MAX_NEIGHBORS) break;

        int rect_idx = std::uniform_int_distribution<>(0, n - 1)(gen);
        const auto& rect = solution[rect_idx];

        if (rect.width == rect.height) continue;

        RectanglePlacement moved = rect;
        moved.rotated = !moved.rotated;

        if (moved.x + moved.get_actual_width() <= L &&
            moved.y + moved.get_actual_height() <= L &&
            !has_collision(moved, moved.box_id, rect_idx)) {
            auto neighbor = solution;
            neighbor[rect_idx] = moved;
            neighbors.push_back(neighbor);
        }
    }

    // STRATEGY 3: If we still have capacity and sparse boxes remain, try direct sparse-to-sparse merging
    if (neighbors.size() < MAX_NEIGHBORS && sparse_boxes.size() >= 2) {
        for (int i = 0; i < std::min(5, (int)sparse_boxes.size()) && neighbors.size() < MAX_NEIGHBORS; i++) {
            for (int j = i + 1; j < std::min(i + 3, (int)sparse_boxes.size()) && neighbors.size() < MAX_NEIGHBORS; j++) {
                int box1 = sparse_boxes[i];
                int box2 = sparse_boxes[j];

                // Check if merging is feasible
                if (box_areas[box1] + box_areas[box2] > L * L * 0.95) continue;

                // Try to move all rectangles from box2 to box1 with random positions
                auto neighbor = solution;
                bool all_placed = true;

                for (int rect_idx : box_rects[box2]) {
                    auto& rect = neighbor[rect_idx];
                    rect.box_id = box1;

                    bool placed = false;
                    for (int attempt = 0; attempt < 5; attempt++) {
                        auto [x, y] = generate_random_position(rect.get_actual_width(), rect.get_actual_height());
                        rect.x = x;
                        rect.y = y;

                        if (!has_collision(rect, box1, rect_idx)) {
                            placed = true;
                            break;
                        }
                    }

                    if (!placed) {
                        all_placed = false;
                        break;
                    }
                }

                if (all_placed) {
                    neighbors.push_back(neighbor);
                }
            }
        }
    }

    //std::cout << "Generated " << neighbors.size() << " neighbors ("
    //          << sparse_boxes.size() << " sparse boxes detected)" << std::endl;
    return neighbors;
}