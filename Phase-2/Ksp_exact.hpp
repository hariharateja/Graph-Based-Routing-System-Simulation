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

json findKsp(const Graph& graph, const json& query);