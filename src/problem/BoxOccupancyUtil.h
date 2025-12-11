//
// Created by arda on 11/24/25.
//

#ifndef OPTIMALGO_BOXOCCUPANCYUTIL_H
#define OPTIMALGO_BOXOCCUPANCYUTIL_H

#include "RectanglePlacement.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <stack>
#include <unordered_map>
#include <set>
#include <memory>
#include <cmath>

class SpatialHash {
    int cell_size;
    int grid_width;
    std::unordered_map<int, std::vector<int>> cells;

    int get_cell_key(int cx, int cy) const {
        return cy * grid_width + cx;
    }

public:
    SpatialHash(int box_length, int avg_rect_size) {
        cell_size = std::max(1, (int)std::round(std::sqrt(avg_rect_size * avg_rect_size)));
        grid_width = (box_length + cell_size - 1) / cell_size + 1;
    }

    void insert(int rect_idx, const RectanglePlacement& r) {
        int x1 = r.x / cell_size;
        int y1 = r.y / cell_size;
        int x2 = (r.x + r.get_actual_width() - 1) / cell_size;
        int y2 = (r.y + r.get_actual_height() - 1) / cell_size;

        for (int cy = y1; cy <= y2; cy++) {
            for (int cx = x1; cx <= x2; cx++) {
                cells[get_cell_key(cx, cy)].push_back(rect_idx);
            }
        }
    }

    std::vector<int> query(const RectanglePlacement& r) const {
        std::set<int> result_set;
        int x1 = r.x / cell_size;
        int y1 = r.y / cell_size;
        int x2 = (r.x + r.get_actual_width() - 1) / cell_size;
        int y2 = (r.y + r.get_actual_height() - 1) / cell_size;

        for (int cy = y1; cy <= y2; cy++) {
            for (int cx = x1; cx <= x2; cx++) {
                auto it = cells.find(get_cell_key(cx, cy));
                if (it != cells.end()) {
                    result_set.insert(it->second.begin(), it->second.end());
                }
            }
        }
        return std::vector<int>(result_set.begin(), result_set.end());
    }
};

struct BoxData {
    std::vector<int> rect_indices;
    long long total_area;
    std::unique_ptr<SpatialHash> hash_grid;
    std::vector<int> occupancy_grid;
    int L;

    BoxData(int box_length, int avg_size) : total_area(0), L(box_length) {
        hash_grid = std::make_unique<SpatialHash>(L, avg_size);
        occupancy_grid.resize(L * L, -1);
    }

    BoxData(BoxData&& other) noexcept = default;
    BoxData& operator=(BoxData&& other) noexcept = default;
    BoxData(const BoxData&) = delete;
    BoxData& operator=(const BoxData&) = delete;
};

inline void build_box_occupancy(BoxData& data, const std::vector<RectanglePlacement>& solution) {
    std::fill(data.occupancy_grid.begin(), data.occupancy_grid.end(), -1);
    int L = data.L;

    for (int rect_idx : data.rect_indices) {
        const auto& r = solution[rect_idx];
        int w = r.get_actual_width();
        int h = r.get_actual_height();

        for (int y = r.y; y < r.y + h; ++y) {
            for (int x = r.x; x < r.x + w; ++x) {
                if (x >= 0 && x < L && y >= 0 && y < L) {
                    data.occupancy_grid[y * L + x] = rect_idx;
                }
            }
        }
    }
}

#endif //OPTIMALGO_BOXOCCUPANCYUTIL_H