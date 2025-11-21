#pragma once
#include<nlohmann/json.hpp>
#include<set>
#include"graph.hpp"
#include <string>
#include <vector>

using json = nlohmann::json;

struct Forbidden{
    std::set<int> forbidden_nodes;
    std::set<std::string> forbidden_road_types;

    void fill_forbidden(const json& query, Forbidden& forbidden);
};

struct Output{
    double min_distance = 0.0;
    double min_time = 0.0;
    bool path_found = false;
};

struct ShortestPathResult {
    int id = -1;
    bool possible = -1;
    double minimum_distance = -1.0;
    double minimum_time = -1.0;
    
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
double calculateTravelTime(const Edge& edge, double startTime);