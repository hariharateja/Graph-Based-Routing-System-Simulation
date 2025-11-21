#pragma once
#include "graph.hpp"
#include "utility.hpp"
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

json findKsp_heuristic(const Graph& graph, const json& query); //ESX-MinW applied yens algorithm to find K shortest paths