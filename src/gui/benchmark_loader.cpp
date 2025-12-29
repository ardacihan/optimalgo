//
// BenchmarkLoader.cpp
//

#include "benchmark_loader.h"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iostream>

std::string BenchmarkLoader::extractTagContent(const std::string& line, const std::string& tag) {
    std::string opening_tag = "<" + tag + ">";
    std::string closing_tag = "</" + tag + ">";

    size_t start = line.find(opening_tag);
    size_t end = line.find(closing_tag);

    if (start == std::string::npos || end == std::string::npos) {
        return "";
    }

    start += opening_tag.length();
    return line.substr(start, end - start);
}

bool BenchmarkLoader::parseBool(const std::string& value) {
    return value == "true" || value == "1";
}

std::optional<LoadedSolution> BenchmarkLoader::load(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return std::nullopt;
    }

    LoadedSolution solution;
    std::string line;

    enum class ParseState {
        NONE,
        METADATA,
        PLACEMENTS,
        RECTANGLE
    };

    ParseState state = ParseState::NONE;
    RectanglePlacement current_rect(0, 0, 0, 0, false, 0);

    while (std::getline(file, line)) {
        // Trim whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.find("<Metadata>") != std::string::npos) {
            state = ParseState::METADATA;
        }
        else if (line.find("</Metadata>") != std::string::npos) {
            state = ParseState::NONE;
        }
        else if (line.find("<Placements>") != std::string::npos) {
            state = ParseState::PLACEMENTS;
        }
        else if (line.find("</Placements>") != std::string::npos) {
            state = ParseState::NONE;
        }
        else if (line.find("<Rectangle>") != std::string::npos) {
            state = ParseState::RECTANGLE;
            current_rect = RectanglePlacement(0, 0, 0, 0, false, 0);
        }
        else if (line.find("</Rectangle>") != std::string::npos) {
            solution.placements.push_back(current_rect);
            state = ParseState::PLACEMENTS;
        }
        else if (state == ParseState::METADATA) {
            std::string content;

            if ((content = extractTagContent(line, "BoxSize")) != "") {
                solution.box_size = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "RectangleCount")) != "") {
                solution.rect_count = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "BoxesUsed")) != "") {
                solution.boxes_used = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "Utilization")) != "") {
                solution.utilization = std::stod(content);
            }
            else if ((content = extractTagContent(line, "SolverName")) != "") {
                solution.solver_name = content;
            }
            else if ((content = extractTagContent(line, "SolveTime")) != "") {
                solution.solve_time = std::stod(content);
            }
            else if ((content = extractTagContent(line, "ObjectiveValue")) != "") {
                solution.objective_value = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "Timestamp")) != "") {
                solution.timestamp = content;
            }
            else if ((content = extractTagContent(line, "MinWidth")) != "") {
                solution.min_width = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "MaxWidth")) != "") {
                solution.max_width = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "MinHeight")) != "") {
                solution.min_height = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "MaxHeight")) != "") {
                solution.max_height = std::stoi(content);
            }
        }
        else if (state == ParseState::RECTANGLE) {
            std::string content;

            if ((content = extractTagContent(line, "Width")) != "") {
                current_rect.width = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "Height")) != "") {
                current_rect.height = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "X")) != "") {
                current_rect.x = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "Y")) != "") {
                current_rect.y = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "BoxID")) != "") {
                current_rect.box_id = std::stoi(content);
            }
            else if ((content = extractTagContent(line, "Rotated")) != "") {
                current_rect.rotated = parseBool(content);
            }
        }
    }

    file.close();

    if (solution.placements.empty()) {
        std::cerr << "No placements found in file: " << filepath << std::endl;
        return std::nullopt;
    }

    return solution;
}

std::vector<std::string> BenchmarkLoader::getAvailableFiles() {
    std::vector<std::string> files;

    try {
        for (const auto& entry : std::filesystem::directory_iterator(".")) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                if (filename.find("solution_") == 0 && filename.ends_with(".xml")) {
                    files.push_back(filename);
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error scanning directory: " << e.what() << std::endl;
    }

    std::sort(files.begin(), files.end());
    return files;
}