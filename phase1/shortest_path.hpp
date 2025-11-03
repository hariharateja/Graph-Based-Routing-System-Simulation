#pragma once
#include<nlohmann/json.hpp>

#include"graph.hpp"
#include <string>
#include <vector>

using json = nlohmann::json;

struct ShortestPathResult {
    int id;
    bool possible;
    double minimum_distance;
    double minimum_time;
    
    std::vector<int> path;
    json toJson(const std::string& mode) const {
        json result;
        result["id"] = id;
        result["possible"] = possible;
        if (possible) {
            if (mode == "distance") {
                result["minimum_distance"] = minimum_distance;
            } else {
                result["minimum_time"] = minimum_time;
            }
            result["path"] = path;
        }
        return result;
    }
};

ShortestPathResult findShortestPath(const Graph& graph, const json& query);
