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

json findAsp(const Graph& graph, const json& query); 

static void asp_dijsktra(const Graph& graph, int source, std::vector<double>& dist);
