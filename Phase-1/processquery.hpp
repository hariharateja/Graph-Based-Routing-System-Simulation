#pragma once
#include "graph.hpp"
#include "shortest_path.hpp"
#include "knn.hpp"

using json = nlohmann::json;
json process_query(const json& query);
// static std::string formatJson(const json& j, int indent , const std::string& key);