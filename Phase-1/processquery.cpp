#include"processquery.hpp"
Graph graph;
json process_query(const json& event){
    std::string eventType = event.at("type");
    if (eventType == "shortest_path") {
        ShortestPathResult result = findShortestPath(graph, event);
        return result.toJson(event.at("mode"));
    }
    else if (eventType == "knn") {
        return findKnn(graph, event);

    }
    else if (eventType == "remove_edge") {
        graph.removeEdge(event.at("edge_id"));
        return json{{"done", true}};

    }
    else if (eventType == "modify_edge") {
        graph.modifyEdge(event.at("edge_id"), event.at("patch"));
        return json{{"done", true}};
    }
    // Unknown event type: return an error JSON
    return json{{"error", "unknown_event_type"}};
}
// static std::string formatJson(const json& j, int indent = 0, const std::string& key = "") {
//     std::string ind(indent, ' ');
//     if (j.is_object()) {
//         std::string out = "{\n";
//         bool first = true;
//         for (auto it = j.begin(); it != j.end(); ++it) {
//             if (!first) out += ",\n";
//             first = false;
//             out += std::string(indent + 4, ' ') + "\"" + it.key() + "\": ";
//             out += formatJson(it.value(), indent + 4, it.key());
//         }
//         out += "\n" + ind + "}";
//         return out;
//     }
//     else if (j.is_array()) {
        
//         if (key == "path" || key == "nodes") {
//         return j.dump();
//         }
//         if (j.empty()) return "[]";
//         std::string out = "[\n";
//         for (size_t i = 0; i < j.size(); ++i) {
//             if (i) out += ",\n";
//             out += std::string(indent + 4, ' ') + formatJson(j[i], indent + 4, "");
//         }
//         out += "\n" + ind + "]";
//         return out;
//     }
//     // primitives and strings: use dump() to get correct quoting
//     return j.dump();
// }
