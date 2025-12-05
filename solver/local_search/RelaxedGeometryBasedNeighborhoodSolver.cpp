#include "RelaxedGeometryBasedNeighborhoodSolver.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <unordered_map>
#include <set>

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_neighbors(
    RectangleFittingProblem &problem, int T) {
    return construct_overlapping_neighbors(problem, T);
}

// Helper: Calculate total overlap area in a solution
long long RelaxedGeometryBasedNeighborhoodSolver::calculate_total_overlap(
    const std::vector<RectanglePlacement>& solution) {

    long long total_overlap = 0;
    int n = solution.size();

    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (solution[i].box_id != solution[j].box_id) continue;

            const auto& r1 = solution[i];
            const auto& r2 = solution[j];

            int overlap_x1 = std::max(r1.x, r2.x);
            int overlap_y1 = std::max(r1.y, r2.y);
            int overlap_x2 = std::min(r1.x + r1.get_actual_width(),
                                     r2.x + r2.get_actual_width());
            int overlap_y2 = std::min(r1.y + r1.get_actual_height(),
                                     r2.y + r2.get_actual_height());

            if (overlap_x1 < overlap_x2 && overlap_y1 < overlap_y2) {
                total_overlap += (long long)(overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1);
            }
        }
    }

    return total_overlap;
}

// Helper: Generate random spread (used at high temperatures)
std::vector<RectanglePlacement>
RelaxedGeometryBasedNeighborhoodSolver::generate_random_spread(
    const std::vector<RectanglePlacement>& solution,
    int L, int T) {

    auto spread_solution = solution;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> pos_dist(0, L-1);

    double spread_factor = T / 1000.0;

    for (auto& rect : spread_solution) {
        rect.x = pos_dist(gen);
        rect.y = pos_dist(gen);

        if (rect.width != rect.height && (rand() % 100) < (T / 10)) {
            rect.rotated = !rect.rotated;
        }

        int w = rect.get_actual_width();
        int h = rect.get_actual_height();
        if (rect.x + w > L) rect.x = L - w;
        if (rect.y + h > L) rect.y = L - h;
    }

    return spread_solution;
}

// Helper: Generate swap move between boxes
std::vector<RectanglePlacement>
RelaxedGeometryBasedNeighborhoodSolver::generate_swap_move(
    const std::vector<RectanglePlacement>& solution,
    int L, int T) {

    auto swapped_solution = solution;
    int n = solution.size();
    if (n < 2) return swapped_solution;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> rect_dist(0, n-1);
    std::uniform_int_distribution<> pos_dist(0, L-1);

    int i = rect_dist(gen);
    int j = rect_dist(gen);
    while (i == j) j = rect_dist(gen);

    auto& rect1 = swapped_solution[i];
    auto& rect2 = swapped_solution[j];

    std::swap(rect1.box_id, rect2.box_id);

    rect1.x = pos_dist(gen);
    rect1.y = pos_dist(gen);
    rect2.x = pos_dist(gen);
    rect2.y = pos_dist(gen);

    if (rect1.width != rect1.height && (rand() % 100) < (T / 20)) {
        rect1.rotated = !rect1.rotated;
    }
    if (rect2.width != rect2.height && (rand() % 100) < (T / 20)) {
        rect2.rotated = !rect2.rotated;
    }

    int w1 = rect1.get_actual_width();
    int h1 = rect1.get_actual_height();
    int w2 = rect2.get_actual_width();
    int h2 = rect2.get_actual_height();

    if (rect1.x + w1 > L) rect1.x = L - w1;
    if (rect1.y + h1 > L) rect1.y = L - h1;
    if (rect2.x + w2 > L) rect2.x = L - w2;
    if (rect2.y + h2 > L) rect2.y = L - h2;

    return swapped_solution;
}

std::vector<std::vector<RectanglePlacement>>
RelaxedGeometryBasedNeighborhoodSolver::construct_overlapping_neighbors(
    RectangleFittingProblem &problem, int T) {

    std::vector<std::vector<RectanglePlacement>> nbs;
    auto solution = problem.get_current_solution();
    int L = problem.get_box_length();
    int n = solution.size();
    if (n == 0) return nbs;

    const int MAX_NEIGHBORS = 2000;

    std::cout << "\n=== RELAXED GEOMETRY SOLVER (T=" << T << ") ===" << std::endl;

    long long initial_overlap = calculate_total_overlap(solution);
    std::cout << "Initial overlap: " << initial_overlap << std::endl;

    // ======================================
    // KEY IMPROVEMENT: Normalize temperature to [0, 1]
    // ======================================
    double temp_ratio = std::clamp(T / 1000.0, 0.0, 1.0);  // 0.0 at T=0, 1.0 at T=1000
    double exploration_factor = temp_ratio;                 // High T = more exploration
    double exploitation_factor = 1.0 - temp_ratio;          // Low T = more exploitation

    std::cout << "Temperature ratio: " << temp_ratio
              << " (exploration: " << exploration_factor
              << ", exploitation: " << exploitation_factor << ")" << std::endl;

    // ======================================
    // PHASE 1: GEOMETRY-BASED MOVES (foundation - always present)
    // ======================================
    auto geometry_neighbors = GeometryBasedNeighborhoodSolver::construct_neighbors(problem, T);

    // At low T: use MORE geometry moves (50%-80%)
    // At high T: use FEWER geometry moves (30%-50%) to leave room for exploration
    double geometry_ratio = 0.3 + 0.5 * exploitation_factor;  // 30% to 80%
    int num_geometry_moves = std::min(
        (int)geometry_neighbors.size(),
        (int)(MAX_NEIGHBORS * geometry_ratio)
    );

    for (int i = 0; i < num_geometry_moves && nbs.size() < MAX_NEIGHBORS; i++) {
        nbs.push_back(geometry_neighbors[i]);
    }

    std::cout << "Added " << nbs.size() << " geometry-based moves (ratio: "
              << geometry_ratio << ")" << std::endl;

    // ======================================
    // PHASE 2: CONSOLIDATION MOVES (smart geometric move - works at ALL temperatures)
    // ======================================
    int consolidation_moves = 0;

    // At LOW temperature: prioritize consolidation (10-30 moves)
    // At HIGH temperature: still do some consolidation (5-10 moves)
    int num_consolidations = (int)(5 + 25 * exploitation_factor);
    num_consolidations = std::min(num_consolidations, MAX_NEIGHBORS - (int)nbs.size());

    // Calculate box occupancies
    std::unordered_map<int, std::vector<int>> box_rects;
    std::unordered_map<int, long long> box_used_area;

    for (int i = 0; i < n; i++) {
        int box_id = solution[i].box_id;
        box_rects[box_id].push_back(i);
        box_used_area[box_id] += (long long)solution[i].width * solution[i].height;
    }

    long long box_capacity = (long long)L * L;

    // Sort boxes by occupancy (sparse to dense)
    std::vector<std::pair<double, int>> box_occupancies;
    for (const auto& [box_id, area] : box_used_area) {
        double occ = (double)area / box_capacity;
        box_occupancies.push_back({occ, box_id});
    }
    std::sort(box_occupancies.begin(), box_occupancies.end());

    // Try moving rectangles from sparse boxes to denser boxes
    for (const auto& [occupancy, sparse_box_id] : box_occupancies) {
        if (consolidation_moves >= num_consolidations) break;
        if (occupancy > 0.7) break;  // Only from sparse boxes

        const auto& sparse_rects = box_rects[sparse_box_id];

        for (int rect_idx : sparse_rects) {
            if (consolidation_moves >= num_consolidations) break;

            const auto& rect = solution[rect_idx];
            long long rect_area = (long long)rect.width * rect.height;

            // Try moving to denser boxes
            for (const auto& [target_occ, target_box_id] : box_occupancies) {
                if (target_box_id == sparse_box_id) continue;

                long long target_area = box_used_area[target_box_id];
                if (target_area + rect_area > box_capacity * 0.95) continue;  // Too full

                // Generate neighbor: move rectangle to target box
                auto neighbor = solution;
                neighbor[rect_idx].box_id = target_box_id;

                // Try different placement positions
                std::vector<std::pair<int, int>> positions = {
                    {0, 0},  // Bottom-left corner
                    {L - rect.get_actual_width(), 0},  // Bottom-right
                    {0, L - rect.get_actual_height()},  // Top-left
                    {L - rect.get_actual_width(), L - rect.get_actual_height()},  // Top-right
                };

                // At low T, also try positions near existing rectangles in target box
                if (exploitation_factor > 0.5) {
                    for (int other_idx : box_rects[target_box_id]) {
                        const auto& other = solution[other_idx];
                        positions.push_back({other.x + other.get_actual_width(), other.y});
                        positions.push_back({other.x, other.y + other.get_actual_height()});
                    }
                }

                for (const auto& [px, py] : positions) {
                    int w = neighbor[rect_idx].get_actual_width();
                    int h = neighbor[rect_idx].get_actual_height();

                    if (px >= 0 && py >= 0 && px + w <= L && py + h <= L) {
                        neighbor[rect_idx].x = px;
                        neighbor[rect_idx].y = py;

                        long long new_overlap = calculate_total_overlap(neighbor);
                        if (new_overlap <= initial_overlap) {
                            nbs.push_back(neighbor);
                            consolidation_moves++;
                            break;
                        }
                    }
                }

                if (consolidation_moves % 5 == 0) break;  // Don't try too many targets per rect
            }
        }
    }

    std::cout << "Added " << consolidation_moves << " consolidation moves" << std::endl;

    // ======================================
    // PHASE 3: EXPLORATION MOVES (temperature-scaled, not thresholded)
    // ======================================
    int exploration_moves = 0;

    // A. Random spread moves - probability based on temperature
    double spread_probability = exploration_factor * exploration_factor;  // Quadratic falloff
    int num_spreads = (int)(5 * spread_probability);
    num_spreads = std::min(num_spreads, MAX_NEIGHBORS - (int)nbs.size());

    for (int i = 0; i < num_spreads; i++) {
        nbs.push_back(generate_random_spread(solution, L, T));
        exploration_moves++;
    }

    // B. Swap moves - linear probability
    int num_swaps = (int)(10 * exploration_factor);
    num_swaps = std::min(num_swaps, MAX_NEIGHBORS - (int)nbs.size());

    for (int i = 0; i < num_swaps; i++) {
        nbs.push_back(generate_swap_move(solution, L, T));
        exploration_moves++;
    }

    std::cout << "Added " << exploration_moves << " exploration moves "
              << "(spreads: " << num_spreads << ", swaps: " << num_swaps << ")" << std::endl;

    // ======================================
    // PHASE 4: OVERLAP RESOLUTION (if overlaps exist)
    // ======================================
    if (initial_overlap > 0) {
        int resolution_moves = 0;
        int max_resolution = std::min(50, MAX_NEIGHBORS - (int)nbs.size());

        // NEW: First try creating a new box for heavily overlapping rectangles
        std::vector<std::pair<long long, int>> rect_overlap_score;
        for (int i = 0; i < n; i++) {
            long long overlap = 0;
            for (int j = 0; j < n; j++) {
                if (i == j || solution[i].box_id != solution[j].box_id) continue;

                const auto& r1 = solution[i];
                const auto& r2 = solution[j];

                int ox1 = std::max(r1.x, r2.x);
                int oy1 = std::max(r1.y, r2.y);
                int ox2 = std::min(r1.x + r1.get_actual_width(), r2.x + r2.get_actual_width());
                int oy2 = std::min(r1.y + r1.get_actual_height(), r2.y + r2.get_actual_height());

                if (ox1 < ox2 && oy1 < oy2) {
                    overlap += (long long)(ox2 - ox1) * (oy2 - oy1);
                }
            }
            if (overlap > 0) {
                rect_overlap_score.push_back({overlap, i});
            }
        }

        // Sort by overlap amount (most overlapped first)
        std::sort(rect_overlap_score.rbegin(), rect_overlap_score.rend());

        // Try moving most overlapped rectangles to new boxes
        int max_new_boxes = std::min(5, MAX_NEIGHBORS - (int)nbs.size());
        for (int i = 0; i < std::min(max_new_boxes, (int)rect_overlap_score.size()); i++) {
            int rect_idx = rect_overlap_score[i].second;

            // Find highest box ID
            int max_box_id = 0;
            for (const auto& r : solution) {
                max_box_id = std::max(max_box_id, r.box_id);
            }

            // Create neighbor with rectangle in new box
            auto neighbor = solution;
            neighbor[rect_idx].box_id = max_box_id + 1;
            neighbor[rect_idx].x = 0;  // Place at origin
            neighbor[rect_idx].y = 0;

            long long new_overlap = calculate_total_overlap(neighbor);
            if (new_overlap < initial_overlap) {
                nbs.push_back(neighbor);
                resolution_moves++;
            }
        }

        std::cout << "Added " << resolution_moves << " new-box moves for overlapped rectangles" << std::endl;

        // Then try resolving overlaps within boxes
        for (const auto& [box_id, rect_indices] : box_rects) {
            if (resolution_moves >= max_resolution) break;

            std::vector<std::pair<int, int>> overlapping_pairs;
            for (size_t i = 0; i < rect_indices.size(); i++) {
                for (size_t j = i + 1; j < rect_indices.size(); j++) {
                    const auto& r1 = solution[rect_indices[i]];
                    const auto& r2 = solution[rect_indices[j]];
                    if (r1.collides(r2)) {
                        overlapping_pairs.push_back({rect_indices[i], rect_indices[j]});
                    }
                }
            }

            for (const auto& [idx1, idx2] : overlapping_pairs) {
                if (resolution_moves >= max_resolution) break;

                const auto& rect1 = solution[idx1];
                const auto& rect2 = solution[idx2];

                for (int attempt = 0; attempt < 5 && resolution_moves < max_resolution; attempt++) {
                    auto neighbor = solution;
                    auto& rect = neighbor[idx1];

                    int center1_x = rect1.x + rect1.get_actual_width() / 2;
                    int center1_y = rect1.y + rect1.get_actual_height() / 2;
                    int center2_x = rect2.x + rect2.get_actual_width() / 2;
                    int center2_y = rect2.y + rect2.get_actual_height() / 2;

                    int dx = center1_x - center2_x;
                    int dy = center1_y - center2_y;

                    double dist = sqrt(dx*dx + dy*dy);
                    if (dist > 0) {
                        dx = (int)(dx / dist * (10 + attempt * 5));
                        dy = (int)(dy / dist * (10 + attempt * 5));
                    }

                    int new_x = std::max(0, std::min(L - rect.get_actual_width(), rect.x + dx));
                    int new_y = std::max(0, std::min(L - rect.get_actual_height(), rect.y + dy));

                    rect.x = new_x;
                    rect.y = new_y;

                    long long new_overlap = calculate_total_overlap(neighbor);
                    if (new_overlap < initial_overlap) {
                        nbs.push_back(neighbor);
                        resolution_moves++;
                        break;
                    }

                    if (rect.width != rect.height) {
                        rect.rotated = !rect.rotated;
                        int w = rect.get_actual_width();
                        int h = rect.get_actual_height();
                        if (rect.x + w > L) rect.x = L - w;
                        if (rect.y + h > L) rect.y = L - h;

                        new_overlap = calculate_total_overlap(neighbor);
                        if (new_overlap < initial_overlap) {
                            nbs.push_back(neighbor);
                            resolution_moves++;
                            break;
                        }
                    }
                }
            }
        }

        std::cout << "Added " << (resolution_moves) << " total overlap resolution moves" << std::endl;
    }

    // ======================================
    // PHASE 5: SMALL PERTURBATIONS (always useful, temperature-scaled)
    // ======================================
    // Sort rectangles by box occupancy (least occupied boxes first)
    std::unordered_map<int, int> box_rect_count;
    for (const auto& r : solution) {
        box_rect_count[r.box_id]++;
    }

    std::vector<int> rect_indices(n);
    for (int i = 0; i < n; i++) rect_indices[i] = i;

    std::sort(rect_indices.begin(), rect_indices.end(), [&](int i, int j) {
        return box_rect_count[solution[i].box_id] < box_rect_count[solution[j].box_id];
    });

    int perturbation_moves = 0;

    // Temperature-scaled perturbations: more at high T, fewer at low T
    int max_perturbations = (int)(10 + 15 * exploration_factor);
    max_perturbations = std::min(max_perturbations, MAX_NEIGHBORS - (int)nbs.size());

    for (int i = 0; i < max_perturbations && i < n; i++) {
        auto neighbor = solution;
        int rect_idx = rect_indices[i];
        auto& rect = neighbor[rect_idx];

        // Step size scales with temperature (smaller at low T)
        int max_step = std::max(1, (int)(1 + 5 * exploration_factor));
        int dx = (rand() % (2 * max_step + 1)) - max_step;
        int dy = (rand() % (2 * max_step + 1)) - max_step;

        int new_x = std::max(0, std::min(L - rect.get_actual_width(), rect.x + dx));
        int new_y = std::max(0, std::min(L - rect.get_actual_height(), rect.y + dy));

        if (new_x != rect.x || new_y != rect.y) {
            rect.x = new_x;
            rect.y = new_y;

            // At low T: only accept improvements; at high T: allow worse moves
            double tolerance = 1.0 + 0.2 * exploration_factor;  // 1.0 to 1.2
            long long new_overlap = calculate_total_overlap(neighbor);
            if (new_overlap <= initial_overlap * tolerance) {
                nbs.push_back(neighbor);
                perturbation_moves++;
            }
        }
    }

    std::cout << "Added " << perturbation_moves << " perturbation moves" << std::endl;
    std::cout << "Total neighbors generated: " << nbs.size() << std::endl;

    return nbs;
}