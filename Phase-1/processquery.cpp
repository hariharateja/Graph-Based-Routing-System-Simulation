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
        bool done = graph.removeEdge(event.at("edge_id"));
        return json{{"id", event.at("id")}, {"done", done}};
    }
    
    else if (eventType == "modify_edge") {   
        bool done = graph.modifyEdge(event);
        return json{{"id", event.value("id", -1)}, {"done", done}};
    }
    
    return json{{"error", "unknown_event_type"}};
}