#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include "problem/BoxOccupancyUtil.h"
#include <cmath>
#include <random>
#include <set>
#include <iostream>

// Static variable to track temperature across calls
static int current_relaxed_temperature = 1000;

// Function to reset temperature (call from visualizer when generating new problems)
void reset_relaxed_temperature() {
    current_relaxed_temperature = 1000;
    std::cout << "Relaxed solver temperature reset to 1000" << std::endl;
}

std::vector<std::vector<RectanglePlacement>> RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem) {

    std::cout << "=== RELAXED SOLVER ACTIVATED ===" << std::endl;
    std::cout << "Current temperature: " << current_relaxed_temperature << std::endl;

    auto neighbors = construct_overlapping_neighbors(problem, current_relaxed_temperature);

    // Gradually decrease temperature for next call
    if (current_relaxed_temperature > 0) {
        current_relaxed_temperature = std::max(0, current_relaxed_temperature - 50);
        std::cout << "Temperature decreased to: " << current_relaxed_temperature << std::endl;
    }

    return neighbors;
}

std::vector<std::vector<RectanglePlacement>> RelaxedGeometryBasedNeighborhoodSolver::construct_overlapping_neighbors(
    RectangleFittingProblem &problem, int T) {

    std::cout << "Constructing overlapping neighbors with T=" << T << std::endl;
    std::cout << "Max overlap ratio: " << calculate_max_overlap_ratio(T) << std::endl;
    std::cout << "Max overlap area: " << calculate_max_overlap_area(T, problem.get_box_length()) << std::endl;

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    int MAX_NEIGHBORS = 100;
    nbs.reserve(MAX_NEIGHBORS);

    int avg_size_sq = 0;
    for (const auto& r : solution) avg_size_sq += r.get_actual_width() * r.get_actual_height();
    avg_size_sq = std::max(1, avg_size_sq / n);

    // Build box data structures
    std::unordered_map<int, BoxData> box_data;
    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        auto it = box_data.find(box_id);
        if (it == box_data.end()) {
            box_data.emplace(std::piecewise_construct,
                            std::forward_as_tuple(box_id),
                            std::forward_as_tuple(L, avg_size_sq));
            it = box_data.find(box_id);
        }
        it->second.rect_indices.push_back(i);
        it->second.total_area += solution[i].get_actual_width() * solution[i].get_actual_height();
        it->second.hash_grid->insert(i, solution[i]);
    }

    // Classify boxes by utilization
    std::unordered_set<int> frozen_boxes;
    std::vector<std::pair<int, double>> box_utilizations;

    for (const auto& [box_id, data] : box_data) {
        double utilization = (double)data.total_area / (double)(L * L);
        box_utilizations.push_back({box_id, utilization});
        if (utilization > 0.9) frozen_boxes.insert(box_id);
    }

    std::sort(box_utilizations.begin(), box_utilizations.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    for (auto& pair : box_data) build_box_occupancy(pair.second, solution);

    // Calculate maximum allowed overlap based on temperature
    double max_overlap_ratio = calculate_max_overlap_ratio(T);
    int max_allowed_overlap_area = calculate_max_overlap_area(T, L);

    std::cout << "Temperature: " << T << ", Max overlap ratio: " << max_overlap_ratio
              << ", Max overlap area: " << max_allowed_overlap_area << std::endl;

    // Relaxed collision check that allows overlaps based on temperature
    auto relaxed_collides = [&](const RectanglePlacement& r, int target_box_id, int moved_idx,
                               const std::vector<RectanglePlacement>& current_sol) {
        int w = r.get_actual_width();
        int h = r.get_actual_height();
        int rect_area = w * h;
        int total_overlap_area = 0;

        for (int i = 0; i < (int)current_sol.size(); i++) {
            if (i == moved_idx) continue;
            if (current_sol[i].box_id != target_box_id) continue;

            const auto& other = current_sol[i];
            int ow = other.get_actual_width();
            int oh = other.get_actual_height();

            // Calculate overlap area
            int overlap_x1 = std::max(r.x, other.x);
            int overlap_y1 = std::max(r.y, other.y);
            int overlap_x2 = std::min(r.x + w, other.x + ow);
            int overlap_y2 = std::min(r.y + h, other.y + oh);

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                int overlap_area = (overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
                total_overlap_area += overlap_area;

                // Early exit if overlap exceeds maximum allowed
                if (total_overlap_area > max_allowed_overlap_area) {
                    return true;
                }

                // Check if overlap ratio exceeds maximum
                double overlap_ratio = (double)overlap_area / (double)rect_area;
                if (overlap_ratio > max_overlap_ratio) {
                    return true;
                }
            }
        }

        return total_overlap_area > max_allowed_overlap_area;
    };

    // Relaxed validity check
    auto is_valid_relaxed_move = [&](const RectanglePlacement& moved, int moved_idx,
                                    const std::vector<RectanglePlacement>& current_sol) {
        // Boundary check remains strict
        if (moved.x < 0 || moved.y < 0 ||
            moved.x + moved.get_actual_width() > L ||
            moved.y + moved.get_actual_height() > L)
            return false;

        // Use relaxed collision check
        return !relaxed_collides(moved, moved.box_id, moved_idx, current_sol);
    };

    auto add_neighbor = [&](std::vector<RectanglePlacement>& nb) {
        if (nbs.size() < MAX_NEIGHBORS) {
            nbs.push_back(std::move(nb));
            return true;
        }
        return false;
    };

    // Generate overlapping positions based on temperature
    auto generate_overlapping_positions = [&](const RectanglePlacement& rect,
                                             const BoxData& target_data,
                                             int num_positions = 20) {
        std::set<std::pair<int, int>> positions;
        int rect_w = rect.get_actual_width();
        int rect_h = rect.get_actual_height();

        // Always include corner positions
        positions.insert({0, 0});
        positions.insert({L - rect_w, 0});
        positions.insert({0, L - rect_h});
        positions.insert({L - rect_w, L - rect_h});

        // Include positions that create controlled overlaps
        int overlap_offset = calculate_overlap_offset(T, std::min(rect_w, rect_h));

        // Sample rectangles from target box to generate overlapping positions
        int sample_size = std::min(8, (int)target_data.rect_indices.size());
        for (int k = 0; k < sample_size; k++) {
            int other_idx = target_data.rect_indices[k];
            const auto& other = solution[other_idx];
            int ow = other.get_actual_width();
            int oh = other.get_actual_height();

            // Standard non-overlapping positions
            positions.insert({other.x + ow, other.y});
            positions.insert({other.x - rect_w, other.y});
            positions.insert({other.x, other.y + oh});
            positions.insert({other.x, other.y - rect_h});

            // Overlapping positions based on temperature
            if (T > 0) {
                // Partial overlap positions
                positions.insert({other.x + ow - overlap_offset, other.y});
                positions.insert({other.x - rect_w + overlap_offset, other.y});
                positions.insert({other.x, other.y + oh - overlap_offset});
                positions.insert({other.x, other.y - rect_h + overlap_offset});

                // Full overlap positions when temperature is high
                if (T > 800) {
                    positions.insert({other.x, other.y});
                    positions.insert({other.x + (ow - rect_w) / 2, other.y + (oh - rect_h) / 2});
                }
            }
        }

        // Add random positions when temperature is high
        if (T > 500) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dist_x(0, std::max(0, L - rect_w));
            std::uniform_int_distribution<> dist_y(0, std::max(0, L - rect_h));

            for (int i = 0; i < 10; i++) {
                positions.insert({dist_x(gen), dist_y(gen)});
            }
        }

        return positions;
    };

    // ==================================================================
    // Strategy 1: RELAXED DENSITY-DRIVEN MOVES (Primary strategy)
    // ==================================================================
    for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
        int sparse_box_id = box_utilizations[i].first;
        double sparse_util = box_utilizations[i].second;

        if (sparse_util > 0.6) break;
        if (frozen_boxes.count(sparse_box_id)) continue;

        auto sparse_it = box_data.find(sparse_box_id);
        if (sparse_it == box_data.end()) continue;
        const auto& sparse_data = sparse_it->second;

        for (int idx : sparse_data.rect_indices) {
            if (nbs.size() >= MAX_NEIGHBORS / 2) break;

            const auto& rect = solution[idx];
            long long rect_area = rect.get_actual_width() * rect.get_actual_height();

            // Target denser boxes
            for (size_t j = box_utilizations.size() - 1; j > i; j--) {
                int target_box_id = box_utilizations[j].first;

                if (frozen_boxes.count(target_box_id)) continue;
                if (target_box_id == sparse_box_id) continue;

                auto target_it = box_data.find(target_box_id);
                if (target_it == box_data.end()) continue;
                const auto& target_data = target_it->second;

                // Relaxed area constraint based on temperature
                if (T < 100 && target_data.total_area + rect_area > (long long)L * L * 0.95)
                    continue;

                auto positions = generate_overlapping_positions(rect, target_data);

                for (const auto& [px, py] : positions) {
                    if (nbs.size() >= MAX_NEIGHBORS / 2) break;

                    for (int rot = 0; rot < 2; rot++) {
                        RectanglePlacement moved = rect;
                        moved.box_id = target_box_id;
                        moved.x = px;
                        moved.y = py;
                        if (rot == 1) moved.rotated = !moved.rotated;

                        auto nb = solution;
                        nb[idx] = moved;

                        if (is_valid_relaxed_move(moved, idx, nb)) {
                            if (!add_neighbor(nb)) break;
                        }
                    }
                }

                if (nbs.size() >= MAX_NEIGHBORS / 2) break;
            }
        }
    }

    // ==================================================================
    // Strategy 2: RELAXED SWAP MOVES
    // ==================================================================
    if (nbs.size() < MAX_NEIGHBORS) {
        for (size_t i = 0; i < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; i++) {
            // Allow swaps between more boxes when temperature is high
            double threshold = (T > 500) ? 0.8 : 0.6;
            if (box_utilizations[i].second > threshold) break;

            for (size_t j = i + 1; j < box_utilizations.size() && nbs.size() < MAX_NEIGHBORS; j++) {
                if (box_utilizations[j].second > threshold) break;

                int box_a = box_utilizations[i].first;
                int box_b = box_utilizations[j].first;

                auto data_a_it = box_data.find(box_a);
                auto data_b_it = box_data.find(box_b);
                if (data_a_it == box_data.end() || data_b_it == box_data.end()) continue;

                const auto& data_a = data_a_it->second;
                const auto& data_b = data_b_it->second;

                // Try swapping pairs of rectangles
                int max_tries = (T > 500) ? 5 : 3;
                int max_tries_a = std::min(max_tries, (int)data_a.rect_indices.size());
                int max_tries_b = std::min(max_tries, (int)data_b.rect_indices.size());

                for (int ia = 0; ia < max_tries_a && nbs.size() < MAX_NEIGHBORS; ia++) {
                    int idx_a = data_a.rect_indices[ia];

                    for (int ib = 0; ib < max_tries_b && nbs.size() < MAX_NEIGHBORS; ib++) {
                        int idx_b = data_b.rect_indices[ib];

                        const auto& rect_a = solution[idx_a];
                        const auto& rect_b = solution[idx_b];

                        // Try swapping with rotation options
                        for (int rot_a = 0; rot_a < 2; rot_a++) {
                            for (int rot_b = 0; rot_b < 2; rot_b++) {
                                RectanglePlacement moved_a = rect_a;
                                RectanglePlacement moved_b = rect_b;

                                moved_a.box_id = box_b;
                                moved_b.box_id = box_a;

                                if (rot_a == 1) moved_a.rotated = !moved_a.rotated;
                                if (rot_b == 1) moved_b.rotated = !moved_b.rotated;

                                auto nb = solution;
                                nb[idx_a] = moved_a;
                                nb[idx_b] = moved_b;

                                if (is_valid_relaxed_move(moved_a, idx_a, nb) &&
                                    is_valid_relaxed_move(moved_b, idx_b, nb)) {
                                    add_neighbor(nb);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ==================================================================
    // Strategy 3: OVERLAPPING POSITION EXPLORATION
    // ==================================================================
    if (T > 0 && nbs.size() < MAX_NEIGHBORS) {
        // Explore overlapping positions within the same box
        for (const auto& [box_id, data] : box_data) {
            if (nbs.size() >= MAX_NEIGHBORS) break;

            for (int idx : data.rect_indices) {
                if (nbs.size() >= MAX_NEIGHBORS) break;

                const auto& rect = solution[idx];
                int rect_w = rect.get_actual_width();
                int rect_h = rect.get_actual_height();

                // Try small perturbations that create overlaps
                int perturbation_range = calculate_perturbation_range(T);
                std::set<std::pair<int, int>> perturbations;

                for (int dx = -perturbation_range; dx <= perturbation_range; dx += std::max(1, perturbation_range/3)) {
                    for (int dy = -perturbation_range; dy <= perturbation_range; dy += std::max(1, perturbation_range/3)) {
                        int new_x = rect.x + dx;
                        int new_y = rect.y + dy;

                        if (new_x >= 0 && new_x + rect_w <= L &&
                            new_y >= 0 && new_y + rect_h <= L) {
                            perturbations.insert({new_x, new_y});
                        }
                    }
                }

                for (const auto& [px, py] : perturbations) {
                    if (nbs.size() >= MAX_NEIGHBORS) break;

                    RectanglePlacement moved = rect;
                    moved.x = px;
                    moved.y = py;

                    auto nb = solution;
                    nb[idx] = moved;

                    if (is_valid_relaxed_move(moved, idx, nb)) {
                        add_neighbor(nb);
                    }
                }
            }
        }
    }

    // Debug: Check for overlaps in generated neighbors
    int overlapping_neighbors = 0;
    for (size_t i = 0; i < nbs.size(); i++) {
        bool has_overlap = false;
        for (size_t j = 0; j < nbs[i].size() && !has_overlap; j++) {
            const auto& rect1 = nbs[i][j];
            for (size_t k = j + 1; k < nbs[i].size() && !has_overlap; k++) {
                const auto& rect2 = nbs[i][k];
                if (rect1.box_id != rect2.box_id) continue;

                if (!(rect1.x >= rect2.x + rect2.get_actual_width() ||
                      rect1.x + rect1.get_actual_width() <= rect2.x ||
                      rect1.y >= rect2.y + rect2.get_actual_height() ||
                      rect1.y + rect1.get_actual_height() <= rect2.y)) {
                    has_overlap = true;
                    overlapping_neighbors++;
                    break;
                }
            }
        }
    }

    std::cout << "Generated " << nbs.size() << " neighbors, " << overlapping_neighbors
              << " contain overlaps" << std::endl;

    return nbs;
}

double RelaxedGeometryBasedNeighborhoodSolver::calculate_max_overlap_ratio(int T) {
    // At T=1000: 100% overlap allowed (1.0)
    // At T=0: 0% overlap allowed (0.0)
    if (T <= 0) return 0.0;
    double max_ratio = 1.0 * (T / 1000.0);  // 100% at T=1000, linear decrease to 0% at T=0
    return std::min(1.0, max_ratio);
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_max_overlap_area(int T, int L) {
    // At T=1000: very large overlap area allowed (25% of box area)
    // At T=0: no overlap allowed
    if (T <= 0) return 0;
    int base_area = (L * L) / 4;  // 25% of box area as reference at T=1000
    return static_cast<int>(base_area * (T / 1000.0));
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_overlap_offset(int T, int rect_size) {
    // Overlap offset decreases with temperature
    if (T <= 0) return 0;
    int max_offset = rect_size;  // Can overlap completely at high temperatures
    return static_cast<int>(max_offset * (T / 1000.0));
}

int RelaxedGeometryBasedNeighborhoodSolver::calculate_perturbation_range(int T) {
    // Perturbation range decreases with temperature
    if (T <= 0) return 0;
    return 5 + static_cast<int>(15 * (T / 1000.0));  // Larger range at high temperatures
}