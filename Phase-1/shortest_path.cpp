#include "shortest_path.hpp"
#include <queue>   
#include <map>     
#include <set>
#include <algorithm>  // Add this for std::reverse

struct CompareCost{
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) {
        return a.first > b.first; 
    }
};

ShortestPathResult findShortestPath(const Graph& graph, const json& query){
    int queryId = query.at("id");
    int start = query.at("source");
    int end = query.at("target");
    std::string mode = query.at("mode");

    std::set<int> forbidden_nodes;
    if(query.contains("constraints") && query["constraints"].contains("forbidden_nodes")){
        for (auto& node_id : query["constraints"]["forbidden_nodes"]) {
            forbidden_nodes.insert(int(node_id));
        }
    }

    std::set<std::string> forbidden_road_types;
    if (query.contains("constraints") && query["constraints"].contains("forbidden_road_types")) {
        for (const auto& road_type : query["constraints"]["forbidden_road_types"]) {
            forbidden_road_types.insert(road_type);
        }
    }

    // Check if start or end is forbidden
    if (forbidden_nodes.count(start) || forbidden_nodes.count(end)) {
        ShortestPathResult result;
        result.id = queryId;
        result.possible = false;
        return result;
    }

    std::map<int, double> cost;
    std::map<int, int> parent;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, CompareCost> pq;

    cost[start] = 0;
    pq.push({0.0, start});

    bool path_found = false;

    while(!pq.empty()){
        double currentCost = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        if (cost.count(currentNode) && currentCost > cost[currentNode]) {
            continue; 
        }
        
        if (currentNode == end) {
            path_found = true;
            break;
        }

        for (int edgeId : graph.getNeighborEdges(currentNode)){
            const Edge& edge = graph.getEdge(edgeId);
            if (forbidden_road_types.count(edge.road_type)) continue;
            
            int neighborNode = (edge.u == currentNode) ? edge.v : edge.u;
            
            if (forbidden_nodes.count(neighborNode)) continue;
            
            double edgeCost = (mode == "distance") ? edge.length : edge.average_time;
            double newCost = currentCost + edgeCost;

            if (!cost.count(neighborNode) || newCost < cost[neighborNode]){
                cost[neighborNode] = newCost;
                parent[neighborNode] = currentNode;
                pq.push({newCost, neighborNode});
            }
        }
    }

    ShortestPathResult result;
    result.id = queryId;
    
    if (path_found){
        result.possible = true;
        result.minimum_distance = (mode == "distance") ? cost[end] : 0.0;
        result.minimum_time = (mode == "time") ? cost[end] : 0.0;
        
        int currentNode = end;
        while (currentNode != start) {
            result.path.push_back(currentNode);
            currentNode = parent[currentNode];
        }
        result.path.push_back(start);
        std::reverse(result.path.begin(), result.path.end());
    }
    else {
        result.possible = false;
    }
    
    return result;
}