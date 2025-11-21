#include "processquery.hpp"
Graph graph;
json process_query(const json& q) {
    std::string type = q.at("type");

    if (type == "k_shortest_paths") {
        return findKsp_exact(graph, q);           // your exact Yen's or similar
    } else if (type == "k_shortest_paths_heuristic") {
        return findKsp_heuristic(graph, q);       // your ESX-MinW heuristic
    } else if (type == "approx_shortest_path") {
        return findAsp(graph, q);                 // your ALT-based ASP
    } else {
        json out;
        out["id"] = q.value("id", -1);
        out["error"] = "unknown_type";
        return out;
    }
}