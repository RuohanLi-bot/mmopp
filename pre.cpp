#include <vector>
#include <utility> // for pair
#include <algorithm> // for minmax_element, remove
#include <cmath>     // for floor, ceil
#include <limits>    // for numeric_limits
#include <iostream>
#include <iomanip> // for setprecision
#include <fstream>  // for ifstream
#include <sstream>  // for stringstream

using namespace std;

// 定义坐标类型，first = 纬度, second = 经度
using LatLon = pair<double, double>;
// 定义网格坐标类型
using GridCoord = pair<int, int>;

// 将经纬度坐标映射到二维网格坐标
vector<GridCoord> mapLatLonToGrid(const vector<LatLon>& coordinates, int padding = 1) {
    if (coordinates.empty()) return {};

    // 确定边界
    auto lat_minmax = minmax_element(coordinates.begin(), coordinates.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.first < b.first; });
    auto lon_minmax = minmax_element(coordinates.begin(), coordinates.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.second < b.second; });

    double min_lat = lat_minmax.first->first;
    double max_lat = lat_minmax.second->first;
    double min_lon = lon_minmax.first->second;
    double max_lon = lon_minmax.second->second;

    // 计算范围
    double lat_span = max_lat - min_lat;
    double lon_span = max_lon - min_lon;

    // 处理所有点重合的极端情况
    if (lat_span == 0 && lon_span == 0) {
        return vector<GridCoord>(coordinates.size(), GridCoord(padding, padding));
    }

    // 确定网格尺寸
    const int MAX_GRID_SIZE = 100;
    int grid_height, grid_width;

    if (lat_span > 0) {
        grid_height = static_cast<int>(ceil(lat_span / (1e-5))) + 2 * padding;
        grid_height = min(grid_height, MAX_GRID_SIZE);
    } else {
        grid_height = 2 * padding + 1;
    }

    if (lon_span > 0) {
        grid_width = static_cast<int>(ceil(lon_span / (1e-5))) + 2 * padding;
        grid_width = min(grid_width, MAX_GRID_SIZE);
    } else {
        grid_width = 2 * padding + 1;
    }

    // 映射每个坐标
    vector<GridCoord> grid_coords;
    grid_coords.reserve(coordinates.size());

    double scale_lat = (lat_span > 0) ? (grid_height - 1 - 2 * padding) / lat_span : 0.0;
    double scale_lon = (lon_span > 0) ? (grid_width - 1 - 2 * padding) / lon_span : 0.0;

    for (const auto& coord : coordinates) {
        int x = (lon_span > 0) ? static_cast<int>(round((coord.second - min_lon) * scale_lon)) + padding : padding;
        int y = (lat_span > 0) ? static_cast<int>(round((max_lat - coord.first) * scale_lat)) + padding : padding;
        
        x = max(padding, min(grid_width - 1 - padding, x));
        y = max(padding, min(grid_height - 1 - padding, y));
        
        grid_coords.emplace_back(x, y);
    }

    return grid_coords;
}

// void preprocess() {
int main() {
    vector<LatLon> locations;
    ifstream file("data/shots.txt");
    
    if (!file.is_open()) {
        cerr << "Error: Cannot open shots.txt file!" << endl;
        return 1;
    }
    
    string line;
    while (getline(file, line)) {
        line.erase(remove(line.begin(), line.end(), '['), line.end());
        line.erase(remove(line.begin(), line.end(), ']'), line.end());
        
        stringstream ss(line);
        string lat_str, lon_str;
        
        if (getline(ss, lat_str, ',') && getline(ss, lon_str)) {
            try {
                locations.emplace_back(stod(lat_str), stod(lon_str));
            } catch (const exception& e) {
                cerr << "Error parsing line: " << line << endl;
            }
        }
    }
    
    if (locations.empty()) {
        cerr << "Error: No valid coordinates found in shots.txt!" << endl;
        return 1;
    }

    cout << fixed << setprecision(6);
    cout << "Original Lat/Lon coordinates from shots.txt:\n";
    for (size_t i = 0; i < locations.size(); ++i) {
        cout << "Point " << (i + 1) << ": ["
             << locations[i].first << ", " << locations[i].second << "]\n";
    }

    auto grid_points = mapLatLonToGrid(locations, 1);

    cout << "\nMapped Grid coordinates (x, y):\n";
    for (size_t i = 0; i < grid_points.size(); ++i) {
        cout << grid_points[i].first << ", " << grid_points[i].second << "\n";
    }

    return 0;
}