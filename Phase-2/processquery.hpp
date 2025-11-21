#pragma once 
#include "Ksp_exact.hpp"
#include "Ksp_heuristic.hpp"
#include "Asp.hpp"

using json = nlohmann::json;
json process_query(const json& query);