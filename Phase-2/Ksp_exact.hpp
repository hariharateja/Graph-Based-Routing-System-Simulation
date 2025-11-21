#pragma once
#include "graph.hpp"
#include "utility.hpp"
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

json findKsp_exact(const Graph& graph, const json& query); //yens algorithm to find K shortest paths