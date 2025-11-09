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
        return json{{"id", event.at("id")}, {"done", true}};

    }
    else if (eventType == "modify_edge") {
        if (!event.contains("patch") || event.at("patch").empty()) {
            return json{{"id", event.at("id")}, {"done", false}};
        
        } else {
        
            graph.modifyEdge(event.at("edge_id"), event.at("patch"));
            return json{{"id", event.at("id")}, {"done", true}};
        }
    }
    return json{{"error", "unknown_event_type"}};
}