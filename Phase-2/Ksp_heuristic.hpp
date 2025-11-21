#pragma once
#include "graph.hpp"
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

struct Path {
    std::vector<int> nodes;  
    std::vector<int> edgeIds;
    double cost = 0.0;
};//to identify a path

json findKsp_heuristic(const Graph& graph, const json& query); //ESX-MinW applied yens algorithm to find K shortest paths

double heuristic(const Graph& graph, int node, int target);//heuristic function for A* - Euclidean distance

static std::string makePathSignature(const Path& p); //to identify unique paths [N:1,4,7|E:10,12]