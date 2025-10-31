//
// InstanceGenerator.cpp
//

#include "InstanceGenerator.h"
#include <random>
#include <algorithm>
#include <iostream>

InstanceGenerator::InstanceGenerator(int L, int min_width, int max_width, int min_height, int max_height)
    : L(L), min_width(min_width), max_width(max_width), min_height(min_height), max_height(max_height) {}

std::vector<RectanglePlacement> InstanceGenerator::generate_rectangles(int num_rectangles) {
    std::vector<RectanglePlacement> rectangle_placements;
    rectangle_placements.reserve(num_rectangles);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> height_dist(min_height, max_height);
    std::uniform_int_distribution<> width_dist(min_width, max_width);

    for (int i = 0; i < num_rectangles; i++) {
        int height = height_dist(gen);
        int width = width_dist(gen);
        rectangle_placements.emplace_back(width, height, 0, 0, false, i);
    }

    return rectangle_placements;
}

// Create a better initial solution using First-Fit Decreasing heuristic
std::vector<RectanglePlacement> InstanceGenerator::create_better_initial_solution(
    std::vector<RectanglePlacement>& rectangles, int box_length) {

    // Sort rectangles by area (largest first)
    std::sort(rectangles.begin(), rectangles.end(), [](const auto& a, const auto& b) {
        long long area_a = (long long)a.width * a.height;
        long long area_b = (long long)b.width * b.height;
        return area_a > area_b;
    });

    std::vector<RectanglePlacement> solution;
    solution.reserve(rectangles.size());

    int current_box = 0;
    std::vector<std::pair<int,int>> available_positions;
    available_positions.push_back({0, 0});

    long long current_box_area = 0;
    long long box_capacity = (long long)box_length * box_length;

    for (auto& rect : rectangles) {
        long long rect_area = (long long)rect.width * rect.height;
        bool placed = false;

        // Try to place in current box
        if (current_box_area + rect_area <= box_capacity * 0.90) {
            // Try each available position
            for (size_t i = 0; i < available_positions.size() && !placed; i++) {
                auto [x, y] = available_positions[i];

                // Try both orientations
                for (int rot = 0; rot < 2 && !placed; rot++) {
                    bool rotated = (rot == 1);
                    int w = rotated ? rect.height : rect.width;
                    int h = rotated ? rect.width : rect.height;

                    if (x + w <= box_length && y + h <= box_length) {
                        RectanglePlacement placed_rect(rect.width, rect.height,
                                                       x, y, rotated, current_box);

                        // Check for collisions
                        bool collides = false;
                        for (const auto& existing : solution) {
                            if (existing.box_id == current_box &&
                                placed_rect.collides(existing)) {
                                collides = true;
                                break;
                            }
                        }

                        if (!collides) {
                            solution.push_back(placed_rect);
                            current_box_area += rect_area;

                            // Add new available positions
                            available_positions.push_back({x + w, y});
                            available_positions.push_back({x, y + h});
                            available_positions.erase(available_positions.begin() + i);

                            placed = true;
                        }
                    }
                }
            }
        }

        // Start new box if couldn't place
        if (!placed) {
            current_box++;
            current_box_area = rect_area;
            available_positions.clear();

            RectanglePlacement placed_rect(rect.width, rect.height, 0, 0, false, current_box);
            solution.push_back(placed_rect);

            available_positions.push_back({rect.width, 0});
            available_positions.push_back({0, rect.height});
        }
    }

    std::cout << "Initial solution uses " << (current_box + 1)
              << " boxes for " << rectangles.size() << " rectangles" << std::endl;

    return solution;
}
