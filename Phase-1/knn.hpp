#pragma once
#include "graph.hpp"
#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

json findKnn(const Graph& graph, const json& query);


json findKnn(const Graph& graph, const json& query);