#pragma once
#include<nlohmann/json.hpp>
#include<set>
#include"graph.hpp"
#include <string>
#include <vector>

using json = nlohmann::json;

struct ShortestPathResult {
    int id = -1;
    bool possible = false;
    double minimum_distance = -1.0;
    double minimum_time = -1.0;
    
    std::vector<int> path;
    json toJson(const std::string& mode) const {
        json result;
        result["id"] = id;
        result["possible"] = possible;
        if (possible) {
            if (mode == "distance") {
                result["minimum_time/minimum_distance"] = minimum_distance;
            } else {
                result["minimum_time/minimum_distance"] = minimum_time;
            }
            result["path"] = path;
        }
        return result;
    }
};

ShortestPathResult findShortestPath(const Graph& graph, const json& query);
double calculateTravelTime(const Edge& edge, double startTime);