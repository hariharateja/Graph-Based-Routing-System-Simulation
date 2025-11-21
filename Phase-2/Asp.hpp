#pragma once
#include "graph.hpp"
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

json findAsp(const Graph& graph, const json& query); 

void asp_dijsktra(const Graph& graph, int source, std::vector<double>& dist);
