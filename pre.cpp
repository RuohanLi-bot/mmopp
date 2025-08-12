#include <vector>
#include <utility>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <random>
#include <queue>
#include <climits>
#include <regex>
#include <set>

#define INPUT_FILE "data1/sz.csv"
#define OUTPUT_FILE "data1/newprobsz4.json"
#define PROCESS_FILE "data1/terminal_output.txt"
// 宏定义：最大网格尺寸
#define MAX_GRID_SIZE 50
// 宏定义：必经点数量
#define YELLOW_AREAS_COUNT 3

//problem中：map[x][y] ~ F[y-1][x-1]

// 宏定义：搜索空间限制策略
// 1: 连通性距离限制（为每个点保留与空间中最接近它且不连通的N个点之间的路径）
#define NEAREST_NEIGHBORS_COUNT 5
// 2: 智能限制（为所有必经点间的路径间创建L形网络）
#define SEARCH_SPACE_STRATEGY 1

using namespace std;

// 定义坐标类型，first = 纬度, second = 经度
using LatLon = pair<double, double>;
// 定义网格坐标类型
using GridCoord = pair<int, int>;

struct SpotInfo {
    LatLon coord;
    double price;
    double duration;
};

// 解析坐标字符串 [lat, lon]
LatLon parseCoordinate(const string& coord_str) {
    string cleaned = coord_str;
    cleaned.erase(remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
    cleaned.erase(remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
    cleaned.erase(remove(cleaned.begin(), cleaned.end(), ' '), cleaned.end());
    
    stringstream ss(cleaned);
    string lat_str, lon_str;
    
    if (getline(ss, lat_str, ',') && getline(ss, lon_str)) {
        try {
            double lat = stod(lat_str);
            double lon = stod(lon_str);
            return {lat, lon};
        } catch (const exception& e) {
            cerr << "Error parsing coordinate: " << coord_str << endl;
            return {0.0, 0.0};
        }
    }
    
    return {0.0, 0.0};
}

double parseDouble(const string& str, double default_value = 0.0) {
    if (str.empty() || str == "TRUE" || str == "FALSE") {
        return default_value;
    }
    try {
        return stod(str);
    } catch (const exception& e) {
        return default_value;
    }
}

vector<SpotInfo> readSpotsFromCSV(const string& filename) {
    vector<SpotInfo> spots;
    ifstream file(filename);
    
    if (!file.is_open()) {
        cerr << "Error: Cannot open " << filename << " !" << endl;
        return spots;
    }
    
    string line;
    getline(file, line);
    
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<string> cells;
        
        while (getline(ss, cell, '\t')) {
            cells.push_back(cell);
        }
        
        if (cells.size() >= 16) {
            SpotInfo spot;
            spot.coord = parseCoordinate(cells[3]);
            spot.price = parseDouble(cells[11], 0);
            spot.duration = parseDouble(cells[15], 0);
            if (spot.coord.first != 0.0 || spot.coord.second != 0.0) 
                spots.push_back(spot);
        }
    }
    return spots;
}

// A*算法寻找两点间最短路径
vector<GridCoord> findShortestPath(const GridCoord& start, const GridCoord& goal, 
                                   const vector<vector<int>>& map, int min_x, int min_y) {
    int grid_width = map[0].size();
    int grid_height = map.size();
    
    int start_x = start.first - min_x;
    int start_y = start.second - min_y;
    int goal_x = goal.first - min_x;
    int goal_y = goal.second - min_y;
    
    if (start_x < 0 || start_x >= grid_width || start_y < 0 || start_y >= grid_height ||
        goal_x < 0 || goal_x >= grid_width || goal_y < 0 || goal_y >= grid_height) {
        cout << "Warning: Points out of bounds - start(" << start_x << "," << start_y 
             << ") goal(" << goal_x << "," << goal_y << ") grid(" << grid_width << "x" << grid_height << ")" << endl;
        return {};
    }
    
    if (map[start_y][start_x] == 1 || map[goal_y][goal_x] == 1) {
        cout << "Warning: Start or goal point is impassable - start(" << start_x << "," << start_y 
             << ") goal(" << goal_x << "," << goal_y << ")" << endl;
        return {};
    }
    
    if (start_x == goal_x && start_y == goal_y) {
        return {start};
    }
    
    priority_queue<pair<int, GridCoord>, vector<pair<int, GridCoord>>, 
                   greater<pair<int, GridCoord>>> pq;
    vector<vector<int>> g_score(grid_height, vector<int>(grid_width, INT_MAX));
    vector<vector<GridCoord>> came_from(grid_height, vector<GridCoord>(grid_width, {-1, -1}));
    
    g_score[start_y][start_x] = 0;
    int f_score = abs(goal_x - start_x) + abs(goal_y - start_y);
    pq.push({f_score, {start_x, start_y}});
    
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    while (!pq.empty()) {
        auto current_pair = pq.top();
        pq.pop();
        
        int current_f = current_pair.first;
        auto current = current_pair.second;
        int current_x = current.first;
        int current_y = current.second;
        
        if (current_x == goal_x && current_y == goal_y) {
            vector<GridCoord> path;
            while (current_x != start_x || current_y != start_y) {
                path.push_back({current_x + min_x, current_y + min_y});
                auto prev = came_from[current_y][current_x];
                current_x = prev.first;
                current_y = prev.second;
            }
            path.push_back({start_x + min_x, start_y + min_y});
            reverse(path.begin(), path.end());
            return path;
        }
        
        for (const auto& direction : directions) {
            int dx = direction.first;
            int dy = direction.second;
            int next_x = current_x + dx;
            int next_y = current_y + dy;
            
            if (next_x >= 0 && next_x < grid_width && next_y >= 0 && next_y < grid_height &&
                map[next_y][next_x] == 0) {
                
                int tentative_g = g_score[current_y][current_x] + 1;
                
                if (tentative_g < g_score[next_y][next_x]) {
                    came_from[next_y][next_x] = {current_x, current_y};
                    g_score[next_y][next_x] = tentative_g;
                    int f = tentative_g + abs(goal_x - next_x) + abs(goal_y - next_y);
                    pq.push({f, {next_x, next_y}});
                }
            }
        }
    }
    
    cout << "Warning: No path found between (" << start_x << "," << start_y 
         << ") and (" << goal_x << "," << goal_y << ")" << endl;
    return {};
}

vector<GridCoord> mapLatLonToGrid(const vector<LatLon>& coordinates, int padding = 1) {
    if (coordinates.empty()) return {};

    auto lat_minmax = std::minmax_element(coordinates.begin(), coordinates.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.first < b.first; });
    auto lon_minmax = std::minmax_element(coordinates.begin(), coordinates.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.second < b.second; });

    double min_lat = lat_minmax.first->first;
    double max_lat = lat_minmax.second->first;
    double min_lon = lon_minmax.first->second;
    double max_lon = lon_minmax.second->second;

    double lat_span = max_lat - min_lat;
    double lon_span = max_lon - min_lon;

    if (lat_span == 0 && lon_span == 0) {
        return vector<GridCoord>(coordinates.size(), GridCoord(padding, padding));
    }

    int grid_height, grid_width;

    if (lat_span > 0) {
        grid_height = static_cast<int>(ceil(lat_span / (1e-4))) + 2 * padding;
        grid_height = std::min(grid_height, MAX_GRID_SIZE);
    } else {
        grid_height = 2 * padding + 1;
    }

    if (lon_span > 0) {
        grid_width = static_cast<int>(ceil(lon_span / (1e-4))) + 2 * padding;
        grid_width = std::min(grid_width, MAX_GRID_SIZE);
    } else {
        grid_width = 2 * padding + 1;
    }

    vector<GridCoord> grid_coords;
    grid_coords.reserve(coordinates.size());

    double scale_lat = (lat_span > 0) ? (grid_height - 1 - 2 * padding) / lat_span : 0.0;
    double scale_lon = (lon_span > 0) ? (grid_width - 1 - 2 * padding) / lon_span : 0.0;

    for (const auto& coord : coordinates) {
        // 直接映射：x对应第一个值（纬度），y对应第二个值（经度），不交换位置
        int x = (lat_span > 0) ? static_cast<int>(round((coord.first - min_lat) * scale_lat)) + padding : padding;
        int y = (lon_span > 0) ? static_cast<int>(round((coord.second - min_lon) * scale_lon)) + padding : padding;
        
        x = std::max(padding, std::min(grid_width - 1 - padding, x));
        y = std::max(padding, std::min(grid_height - 1 - padding, y));
        
        grid_coords.emplace_back(x, y);
    }

    return grid_coords;
}

void generateProblemJson(const vector<SpotInfo>& spots, const vector<GridCoord>& grid_points, ofstream& output_file) {
    if (grid_points.empty()) {
        cerr << "Error: grid_points is empty!" << endl;
        return;
    }

    auto x_minmax = std::minmax_element(grid_points.begin(), grid_points.end(),
                                   [](const GridCoord& a, const GridCoord& b) { return a.first < b.first; });
    auto y_minmax = std::minmax_element(grid_points.begin(), grid_points.end(),
                                   [](const GridCoord& a, const GridCoord& b) { return a.second < b.second; });

    int min_x = x_minmax.first->first;
    int max_x = x_minmax.second->first;
    int min_y = y_minmax.first->second;
    int max_y = y_minmax.second->second;

    int grid_width = max_x - min_x + 1;
    int grid_height = max_y - min_y + 1;

    vector<vector<int>> map(grid_height, vector<int>(grid_width, 1));

    GridCoord start_point = grid_points.front();
    GridCoord goal_point = grid_points.front();

    int start_x = start_point.first - min_x + 1;
    int start_y = start_point.second - min_y + 1;
    int goal_x = goal_point.first - min_x + 1;
    int goal_y = goal_point.second - min_y + 1;

    vector<vector<int>> yellow_areas;
    vector<GridCoord> selected_points;
    if (grid_points.size() > 2) {
        random_device rd;
        mt19937 gen(rd());
        
        vector<GridCoord> available_points;
        for (size_t i = 1; i < grid_points.size() - 1; ++i) 
            available_points.push_back(grid_points[i]);
        
        shuffle(available_points.begin(), available_points.end(), gen);
        
        int points_to_select = min(YELLOW_AREAS_COUNT, static_cast<int>(available_points.size()));
        for (int i = 0; i < points_to_select; ++i) {
            selected_points.push_back(available_points[i]);
            int x = available_points[i].first - min_x + 1;
            int y = available_points[i].second - min_y + 1;
            yellow_areas.push_back({x, y});
        }
    }
    map[start_y - 1][start_x - 1] = 0;
    map[goal_y - 1][goal_x - 1] = 0;
    for (const auto& grid_point : grid_points) {
        int x = grid_point.first - min_x + 1;
        int y = grid_point.second - min_y + 1;
        map[y - 1][x - 1] = 0;
    }
    if (SEARCH_SPACE_STRATEGY == 1) {
        // 策略1: 为每个点保留与空间中最接近它且不连通的N个点之间的路径
        for (size_t i = 0; i < grid_points.size(); ++i) {
            GridCoord current_point = grid_points[i];
            vector<pair<double, size_t>> distances;
            
            // 计算当前点到所有其他点的距离
            for (size_t j = 0; j < grid_points.size(); ++j) 
                if (i != j) {
                    GridCoord other_point = grid_points[j];
                    double distance = sqrt(pow(current_point.first - other_point.first, 2) + 
                                        pow(current_point.second - other_point.second, 2));
                    distances.emplace_back(distance, j);
                }
            
            // 按距离排序
            sort(distances.begin(), distances.end());
            
            // 检查连通性并选择不连通的最近N个点
            int points_connected = 0;
            for (const auto& [distance, target_idx] : distances) {
                if (points_connected >= NEAREST_NEIGHBORS_COUNT) break;
                
                GridCoord target_point = grid_points[target_idx];
                
                // 检查两点是否已经连通（通过检查路径上的点）
                bool is_connected = false;
                
                // 转换为网格坐标
                int x1 = current_point.first - min_x + 1;
                int y1 = current_point.second - min_y + 1;
                int x2 = target_point.first - min_x + 1;
                int y2 = target_point.second - min_y + 1;
                
                // 检查L形路径上的点是否已经可通行
                if (x1 != x2) {
                    int step = (x2 > x1) ? 1 : -1;
                    for (int x = x1; x != x2; x += step) 
                        if (x >= 1 && x <= grid_width && y1 >= 1 && y1 <= grid_height) 
                            if (map[y1 - 1][x - 1] == 1) {
                                is_connected = false;
                                break;
                            }
                        // 检查垂直段
                        if (is_connected && y1 != y2) {
                            int step_y = (y2 > y1) ? 1 : -1;
                            for (int y = y1; y != y2; y += step_y) 
                                if (x2 >= 1 && x2 <= grid_width && y >= 1 && y <= grid_height) 
                                    if (map[y - 1][x2 - 1] == 1) {
                                        is_connected = false;
                                        break;
                                    }
                        }
                } else if (y1 != y2) {
                    // 只有垂直移动
                    int step = (y2 > y1) ? 1 : -1;
                    for (int y = y1; y != y2; y += step) 
                        if (x1 >= 1 && x1 <= grid_width && y >= 1 && y <= grid_height) 
                            if (map[y - 1][x1 - 1] == 1) {
                                is_connected = false;
                                break;
                            }
                }
                
                // 如果两点不连通，则创建连接路径
                if (!is_connected) {
                    // 创建L形路径连接
                    if (x1 != x2) {
                        int step = (x2 > x1) ? 1 : -1;
                        for (int x = x1; x != x2; x += step) 
                            if (x >= 1 && x <= grid_width && y1 >= 1 && y1 <= grid_height) 
                                map[y1 - 1][x - 1] = 0;
                    }
                    if (y1 != y2) {
                        int step = (y2 > y1) ? 1 : -1;
                        for (int y = y1; y != y2; y += step) 
                            if (x2 >= 1 && x2 <= grid_width && y >= 1 && y <= grid_height) 
                                map[y - 1][x2 - 1] = 0;
                    }
                    points_connected++;
                }
            }
        }
    } else if (SEARCH_SPACE_STRATEGY == 2) {
        output_file << "Creating paths between all " << grid_points.size() << " points..." << endl;
        
        for (size_t i = 0; i < grid_points.size(); ++i) {
            for (size_t j = i + 1; j < grid_points.size(); ++j) {
                GridCoord p1 = grid_points[i];
                GridCoord p2 = grid_points[j];
                
                int x1 = p1.first - min_x + 1;
                int y1 = p1.second - min_y + 1;
                int x2 = p2.first - min_x + 1;
                int y2 = p2.second - min_y + 1;
                
                if (x1 != x2) {
                    int step = (x2 > x1) ? 1 : -1;
                    for (int x = x1; x != x2; x += step) {
                        if (x >= 1 && x <= grid_width && y1 >= 1 && y1 <= grid_height) {
                            map[y1 - 1][x - 1] = 0;
                        }
                    }
                }
                if (y1 != y2) {
                    int step = (y2 > y1) ? 1 : -1;
                    for (int y = y1; y != y2; y += step) {
                        if (x2 >= 1 && x2 <= grid_width && y >= 1 && y <= grid_height) {
                            map[y - 1][x2 - 1] = 0;
                        }
                    }
                }
            }
        }
    }

    ofstream json_file(OUTPUT_FILE);
    if (!json_file.is_open()) {
        cerr << "Error: Cannot create " << OUTPUT_FILE << " file!" << endl;
        return;
    }
    json_file << "{\n";
    json_file << "  \"Map\": [\n";
    for (size_t i = 0; i < map.size(); ++i) {
        json_file << "    [";
        for (size_t j = 0; j < map[i].size(); ++j) {
            json_file << map[i][j];
            if (j < map[i].size() - 1) json_file << ", ";
        }
        json_file << "]";
        if (i < map.size() - 1) json_file << ",";
        json_file << "\n";
    }
    
    json_file << "  ],\n";
    json_file << "  \"START_x\": " << start_x << ",\n";
    json_file << "  \"START_y\": " << start_y << ",\n";
    json_file << "  \"GOAL_x\": " << goal_x << ",\n";
    json_file << "  \"GOAL_y\": " << goal_y << ",\n";
    
    json_file << "  \"Yellow_areas\": [\n";
    for (size_t i = 0; i < yellow_areas.size(); ++i) {
        json_file << "    [" << yellow_areas[i][0] << ", " << yellow_areas[i][1] << "]";
        if (i < yellow_areas.size() - 1) json_file << ",";
        json_file << "\n";
    }
    json_file << "  ],\n";

    json_file << "  \"F\": [";
    set<pair<int, int>> f_points;
    bool first = true;
    for (size_t i = 0; i < spots.size(); ++i) {
        int x = grid_points[i].first - min_x + 1;
        int y = grid_points[i].second - min_y + 1;
        f_points.insert({x, y});
        if (!first) json_file << ", ";
        json_file << "[" << x << ", " << y << ", " << spots[i].price << ", " << spots[i].duration << "]";
        first = false;
    }
    json_file << "]\n" << "}";
    json_file.close();
    
    output_file << "Generated " << OUTPUT_FILE << " with:" << endl;
    output_file << "  Grid size: " << grid_width << " x " << grid_height << endl;
    output_file << "  Start point: (" << start_x << ", " << start_y << ")" << endl;
    output_file << "  Goal point: (" << goal_x << ", " << goal_y << ")" << endl;
    output_file << "  Total spots: " << spots.size() << endl;
    output_file << "  Yellow areas (must-visit points): " << yellow_areas.size() << endl;
    for (size_t i = 0; i < yellow_areas.size(); ++i) 
        output_file << "    Point " << (i + 1) << ": (" << yellow_areas[i][0] << ", " << yellow_areas[i][1] << ")" << endl;

    switch (SEARCH_SPACE_STRATEGY) {
        case 1:
            output_file << "  Search space: L-shaped paths to " << NEAREST_NEIGHBORS_COUNT << " nearest points for each location" << endl;
            break;
        case 2:
            output_file << "  Search space: All paths between must-visit points" << endl;
            break;
    }
}

void preprocess() {
    vector<SpotInfo> spots = readSpotsFromCSV(INPUT_FILE);
    
    if (spots.empty()) {
        cerr << "Error: No valid spots found in " << INPUT_FILE << "!" << endl;
        return;
    }

    vector<LatLon> locations;
    for (const auto& spot : spots) 
        locations.push_back(spot.coord);

    auto lat_minmax = std::minmax_element(locations.begin(), locations.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.first < b.first; });
    auto lon_minmax = std::minmax_element(locations.begin(), locations.end(),
                                    [](const LatLon& a, const LatLon& b) { return a.second < b.second; });

    auto grid_points = mapLatLonToGrid(locations, 1);
    
    ofstream points_file(PROCESS_FILE);
    if (!points_file.is_open()) {
        cerr << "Error: Cannot create " << PROCESS_FILE << " file!" << endl;
        return;
    }
    
    points_file << fixed << setprecision(6);
    points_file << "Original Lat/Lon coordinates from " << INPUT_FILE << ":\n";
    for (size_t i = 0; i < locations.size(); ++i) {
        points_file << "Point " << (i + 1) << ": ["
             << locations[i].first << ", " << locations[i].second << "]\n";
    }
    points_file << "\nMapped Grid coordinates (x, y):\n";
    for (size_t i = 0; i < grid_points.size(); ++i) 
        points_file << "Point " << (i + 1) << ": ("
             << grid_points[i].first << ", " << grid_points[i].second << ")\n";

    points_file << "\nSpot information:\n";
    for (size_t i = 0; i < spots.size(); ++i) {
        points_file << "Spot " << (i + 1) << ":\n";
        points_file << "  Price: " << spots[i].price << "\n";
        points_file << "  Duration: " << spots[i].duration << "\n";
    }
    generateProblemJson(spots, grid_points, points_file);
    points_file.close();
}

int main() {
    preprocess();
    return 0;
}