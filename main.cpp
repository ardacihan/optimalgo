#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include "RectangleFittingProblem.h"
#include "RectanglePlacement.h"
#include "InstanceGenerator.h"
#include "OptimizationProblem.h"
#include <chrono>
#include "Solver.h"

void visualizeBox(int box_id, const std::vector<RectanglePlacement>& placements, int L) {
    std::cout << "\nBounding Box " << box_id << " (Size: " << L << "x" << L << "):" << std::endl;

    std::vector<std::vector<char>> grid(L, std::vector<char>(L, '.'));

    char current_char = 'A';
    std::map<int, char> rect_chars;

    for (const auto& rect : placements) {
        if (rect.box_id != box_id) continue;

        if (rect_chars.find(rect.width * 1000 + rect.height * 10 + (rect.rotated ? 1 : 0)) == rect_chars.end()) {
            rect_chars[rect.width * 1000 + rect.height * 10 + (rect.rotated ? 1 : 0)] = current_char++;
        }
        char rect_char = rect_chars[rect.width * 1000 + rect.height * 10 + (rect.rotated ? 1 : 0)];

        int actual_width = rect.get_actual_width();
        int actual_height = rect.get_actual_height();

        for (int i = rect.y; i < rect.y + actual_height && i < L; ++i) {
            for (int j = rect.x; j < rect.x + actual_width && j < L; ++j) {
                if (i >= 0 && j >= 0) {
                    grid[i][j] = rect_char;
                }
            }
        }
    }

    std::cout << "   ";
    for (int j = 0; j < L; ++j) {
        std::cout << j % 10;
    }
    std::cout << std::endl;

    for (int i = 0; i < L; ++i) {
        std::cout << i % 10 << "| ";
        for (int j = 0; j < L; ++j) {
            std::cout << grid[i][j];
        }
        std::cout << " |" << std::endl;
    }

    std::cout << "\nLegend:" << std::endl;
    for (const auto& [key, ch] : rect_chars) {
        int w = key / 1000;
        int h = (key % 1000) / 10;
        bool rotated = (key % 10) == 1;
        std::cout << "  " << ch << ": " << w << "x" << h;
        if (rotated) std::cout << " (rotated)";
        std::cout << std::endl;
    }
}

void printRectangleDetails(const std::vector<RectanglePlacement>& placements) {
    std::cout << "\nDetailed Rectangle Placements:" << std::endl;
    std::cout << "==============================" << std::endl;

    std::map<int, std::vector<RectanglePlacement>> boxes;
    for (const auto& rect : placements) {
        boxes[rect.box_id].push_back(rect);
    }

    for (const auto& [box_id, rects] : boxes) {
        std::cout << "\nBox " << box_id << " contains " << rects.size() << " rectangles:" << std::endl;
        for (const auto& rect : rects) {
            std::cout << "  - " << rect.width << "x" << rect.height;
            if (rect.rotated) std::cout << " (rotated → " << rect.get_actual_width() << "x" << rect.get_actual_height() << ")";
            std::cout << " at (" << rect.x << "," << rect.y << ")" << std::endl;
        }
    }
}

int main() {
    int L = 15;
    int num_rectangles = 100;
    int max_steps = 800;
    int N = 1;

    InstanceGenerator instance_generator(L, 1, 15, 1, 15);
    std::vector<RectanglePlacement> rectangles = instance_generator.generate_rectangles(num_rectangles);
    rectangles = instance_generator.create_better_initial_solution(rectangles,L);

    std::cout << "Generated " << rectangles.size() << " random rectangles" << std::endl;

    GeometryBasedNeighborhoodSolver solver;

    std::vector<RectanglePlacement> best_solution;
    int best_obj = std::numeric_limits<int>::min();
    double best_time = 0.0;

    for (int run = 1; run <= N; ++run) {
        std::cout << "\n========== Run " << run << " ==========" << std::endl;

        auto rect_copy = rectangles;
        std::vector<RectanglePlacement> initial_solution =
            instance_generator.create_better_initial_solution(rect_copy, L);

        RectangleFittingProblem problem(L, initial_solution);

        std::cout << "Initial objective: " << problem.objective(initial_solution) << std::endl;

        auto start = std::chrono::steady_clock::now();
        std::vector<RectanglePlacement> solution = solver.solve(problem, max_steps);
        auto end = std::chrono::steady_clock::now();

        int obj = problem.objective(solution);
        std::chrono::duration<double> elapsed_seconds = end - start;

        std::cout << "Run " << run << ": Final Objective = " << obj
                  << ", Time = " << elapsed_seconds.count() << " s" << std::endl;

        if (obj > best_obj) {
            best_obj = obj;
            best_solution = solution;
            best_time = elapsed_seconds.count();
        }
    }

    RectangleFittingProblem best_problem(L, best_solution);
    std::unordered_map<int,int> coverage_per_box = best_problem.get_coverage_each_bounding_box();

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "BEST SOLUTION SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    std::cout << "\nCoverage per box:" << std::endl;
    for (const auto& [box_id, coverage] : coverage_per_box) {
        int percentage = (coverage * 100) / (L * L);
        std::cout << "Box " << box_id << ": " << coverage << "/" << (L * L)
                  << " (" << percentage << "%)" << std::endl;
    }
    std::cout << "\nTotal boxes used: " << coverage_per_box.size() << std::endl;
    std::cout << "Best objective: " << best_obj << std::endl;
    std::cout << "Best run time: " << best_time << " seconds" << std::endl;

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "VISUALIZATION OF BEST SOLUTION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    std::map<int, std::vector<RectanglePlacement>> boxes;
    for (const auto& rect : best_solution) {
        boxes[rect.box_id].push_back(rect);
    }

    printRectangleDetails(best_solution);

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "BOX LAYOUT VISUALIZATION" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    for (const auto& [box_id, placements] : boxes) {
        visualizeBox(box_id, placements, L);
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SUMMARY STATISTICS" << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    int total_area_used = 0;
    int total_rectangles = 0;

    for (const auto& [box_id, placements] : boxes) {
        int box_area_used = 0;
        for (const auto& rect : placements) {
            box_area_used += rect.get_actual_width() * rect.get_actual_height();
        }
        total_area_used += box_area_used;
        total_rectangles += placements.size();

        std::cout << "Box " << box_id << ": " << placements.size() << " rectangles, "
                  << box_area_used << "/" << (L*L) << " area used ("
                  << (box_area_used * 100) / (L*L) << "%)" << std::endl;
    }

    std::cout << "\nTotal: " << total_rectangles << " rectangles in " << boxes.size()
              << " boxes, " << total_area_used << "/" << (boxes.size() * L * L)
              << " total area used (" << (total_area_used * 100) / (boxes.size() * L * L) << "%)" << std::endl;

    return 0;
}