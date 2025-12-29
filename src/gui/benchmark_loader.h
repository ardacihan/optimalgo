//
// BenchmarkLoader.h
//

#ifndef BENCHMARKLOADER_H
#define BENCHMARKLOADER_H

#include <string>
#include <vector>
#include <optional>
#include "../problem/primitives/RectanglePlacement.h"

struct LoadedSolution {
    // Metadata
    int box_size;
    int rect_count;
    int boxes_used;
    double utilization;
    std::string solver_name;
    double solve_time;
    int objective_value;
    std::string timestamp;
    int min_width;
    int max_width;
    int min_height;
    int max_height;

    // Placements
    std::vector<RectanglePlacement> placements;
};

class BenchmarkLoader {
public:
    // Load a solution from an XML file
    static std::optional<LoadedSolution> load(const std::string& filepath);

    // Get list of available XML files in the current directory
    static std::vector<std::string> getAvailableFiles();

private:
    // Helper function to extract text content between XML tags
    static std::string extractTagContent(const std::string& line, const std::string& tag);

    // Helper function to parse boolean from XML
    static bool parseBool(const std::string& value);
};

#endif // BENCHMARKLOADER_H