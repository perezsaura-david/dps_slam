#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>
#include <utility>

namespace elevation_grid_filter {

// Hash function for grid cell indices
struct GridHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        auto h1 = std::hash<int>{}(p.first);
        auto h2 = std::hash<int>{}(p.second);
        return h1 ^ (h2 << 1); // Combine hashes
    }
};

// Store data for each grid cell
struct CellData {
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();
    int point_count = 0;
};

class ElevationGridFilter {

public:
    ElevationGridFilter(float grid_size, float z_diff_threshold, int point_count_threshold)
        : grid_size_(grid_size), z_diff_threshold_(z_diff_threshold), point_count_threshold_(point_count_threshold) {}

    void setParameters(float grid_size, float z_diff_threshold, int point_count_threshold) {
        grid_size_ = grid_size;
        z_diff_threshold_ = z_diff_threshold;
        point_count_threshold_ = point_count_threshold;
    }

    void filter(const std::vector<float>& in_x, const std::vector<float>& in_y, const std::vector<float>& in_z,
                std::vector<float>& out_x, std::vector<float>& out_y, std::vector<float>& out_z) {
        out_x.clear();
        out_y.clear();
        out_z.clear();

        if (in_x.empty() || in_y.empty() || in_z.empty() || in_x.size() != in_y.size() || in_x.size() != in_z.size()) {
            return; // Invalid input
        }

        std::unordered_map<std::pair<int, int>, CellData, GridHash> grid;

        // Fill grid cells and find min/max z for each cell
        for (size_t i = 0; i < in_x.size(); ++i) {
            int idx_x = static_cast<int>(std::floor(in_x[i] / grid_size_));
            int idx_y = static_cast<int>(std::floor(in_y[i] / grid_size_));
            auto key = std::make_pair(idx_x, idx_y);

            grid[key].z_min = std::min(grid[key].z_min, in_z[i]);
            grid[key].z_max = std::max(grid[key].z_max, in_z[i]);
            grid[key].point_count++;
        }

        // Extract points
        for (size_t i = 0; i < in_x.size(); ++i) {
            int idx_x = static_cast<int>(std::floor(in_x[i] / grid_size_));
            int idx_y = static_cast<int>(std::floor(in_y[i] / grid_size_));
            auto key = std::make_pair(idx_x, idx_y);

            if ((grid[key].z_max - grid[key].z_min) >= z_diff_threshold_ && grid[key].point_count > point_count_threshold_) {
                out_x.push_back(in_x[i]);
                out_y.push_back(in_y[i]);
                out_z.push_back(in_z[i]);
            }
        }
    }

private:
    float grid_size_; // mm
    float z_diff_threshold_; // mm
    int point_count_threshold_; // Minimum number of points in a cell to be considered valid
};

}