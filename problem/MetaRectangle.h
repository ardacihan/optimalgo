// MetaRectangle.h
#ifndef OPTIMALGO_METARECTANGLE_H
#define OPTIMALGO_METARECTANGLE_H

#include "RectanglePlacement.h"
#include <vector>
#include <memory>

class MetaRectangle {
private:
    std::vector<RectanglePlacement> contained_rects;
    int bounding_width;
    int bounding_height;
    int total_area;
    double density;
    bool calculated_properties;

public:
    MetaRectangle(const std::vector<RectanglePlacement>& rects)
        : contained_rects(rects), calculated_properties(false) {
        calculate_properties();
    }

    void calculate_properties() {
        if (contained_rects.empty()) {
            bounding_width = 0;
            bounding_height = 0;
            total_area = 0;
            density = 0.0;
            calculated_properties = true;
            return;
        }

        // Calculate bounding box
        int min_x = INT_MAX, min_y = INT_MAX;
        int max_x = 0, max_y = 0;
        total_area = 0;

        for (const auto& rect : contained_rects) {
            int right = rect.x + rect.get_actual_width();
            int bottom = rect.y + rect.get_actual_height();

            min_x = std::min(min_x, rect.x);
            min_y = std::min(min_y, rect.y);
            max_x = std::max(max_x, right);
            max_y = std::max(max_y, bottom);

            total_area += rect.get_actual_width() * rect.get_actual_height();
        }

        bounding_width = max_x - min_x;
        bounding_height = max_y - min_y;

        // Calculate density (area utilization within bounding box)
        long long bounding_area = (long long)bounding_width * bounding_height;
        density = bounding_area > 0 ? (double)total_area / bounding_area : 0.0;

        calculated_properties = true;
    }

    // Getters
    int get_width() const { return bounding_width; }
    int get_height() const { return bounding_height; }
    double get_density() const { return density; }
    int get_total_area() const { return total_area; }
    const std::vector<RectanglePlacement>& get_contained_rects() const { return contained_rects; }

    // Check if this meta-rectangle should be treated as fixed (high density)
    bool should_be_fixed(double density_threshold = 0.85) const {
        return density >= density_threshold;
    }

    // Create a placement for this meta-rectangle
    RectanglePlacement as_placement(int box_id, int x, int y, bool allow_rotation = true) const {
        // For dense groups, we can consider rotating the entire group
        bool rotated = allow_rotation && (bounding_height > bounding_width);
        int actual_width = rotated ? bounding_height : bounding_width;
        int actual_height = rotated ? bounding_width : bounding_height;

        return RectanglePlacement(actual_width, actual_height, x, y, rotated, box_id);
    }

    // Check if this meta-rectangle can fit in a box of given size
    bool can_fit_in(int box_size, bool allow_rotation = true) const {
        if (allow_rotation) {
            return (bounding_width <= box_size && bounding_height <= box_size) ||
                   (bounding_height <= box_size && bounding_width <= box_size);
        }
        return bounding_width <= box_size && bounding_height <= box_size;
    }
};

#endif // OPTIMALGO_METARECTANGLE_H