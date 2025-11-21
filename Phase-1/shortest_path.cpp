#include "shortest_path.hpp"
#include <queue>   
#include <map>     
#include <set>
#include <algorithm>  // Add this for std::reverse

double get_heuristic(const Graph& graph, int u, int target, const std::string& mode){
    const Node& n_u = graph.getNode(u);
    const Node& n_t = graph.getNode(target);

    const double METERS_PER_DEGREE = 111000.0;

    double dx = n_u.lat - n_t.lat;
    double dy = n_u.lon - n_t.lon;
    double dist_meters = std::sqrt(dx*dx + dy*dy) * METERS_PER_DEGREE;

    if (mode == "distance"){
        return dist_meters;
    }
    else{
        const double MAX_SPEED_MPS = 36.0; 
        return dist_meters / MAX_SPEED_MPS;
    }
}

struct CompareCost{
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) {
        return a.first > b.first; 
    }
};
double calculateTravelTime(const Edge& edge, double startTime) {
    if (edge.speed_profile.size() != 96) {
        return edge.average_time;
    }
    
    double remainingDistance = edge.length;
    double currentTime = startTime;
    double totalTime = 0.0;
    
    while (remainingDistance > 0.0001) { 
        long timeOfDay = static_cast<long>(currentTime) % 86400;
        int idx = timeOfDay / 900;
        
        double speed_ms = edge.speed_profile[idx];
        
        long timeInCurrentSlot = timeOfDay % 900;
        double timeRemainingInSlot = 900.0 - timeInCurrentSlot;
        
        double distanceInSlot = speed_ms * timeRemainingInSlot;
        
        if (distanceInSlot >= remainingDistance) {
            totalTime += remainingDistance / speed_ms;
            remainingDistance = 0.0;
        } else {
            totalTime += timeRemainingInSlot;
            remainingDistance -= distanceInSlot;
            currentTime += timeRemainingInSlot;
        }
    }
    
    return totalTime;
}

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

    std::map<int, double> g_score;
    std::map<int, int> parent;
    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, CompareCost> pq;

    g_score[start] = 0.0;
    double start_f = get_heuristic(graph, start, end, mode);
    pq.push({start_f, start});

    bool path_found = false;

    while(!pq.empty()){
        double current_f = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (g_score.count(u) && current_f > g_score[u] + get_heuristic(graph, u, end, mode) + 0.0001) {
             continue; 
        }
        
        if (u == end) {
            path_found = true;
            break;
        }

        for (int edgeId : graph.getNeighborEdges(u)){
            const Edge& edge = graph.getEdge(edgeId);
            if (forbidden_road_types.count(edge.road_type)) continue;
            
            int v = (edge.u == u) ? edge.v : edge.u;
            
            if (forbidden_nodes.count(v)) continue;
            
            // --- Calculate Edge Cost ---
            double edgeCost = 0.0;
            if (mode == "distance") {
                edgeCost = edge.length;
            } else {
                // Crucial: Pass g_score[u] (current time), NOT current_f
                edgeCost = calculateTravelTime(edge, g_score[u]); 
            }
            
            double tentative_g = g_score[u] + edgeCost;

            // If a shorter path to v is found
            if (!g_score.count(v) || tentative_g < g_score[v]){
                g_score[v] = tentative_g;
                parent[v] = u;
                
                // A* Priority = New G + Heuristic to Target
                double f_new = tentative_g + get_heuristic(graph, v, end, mode);
                pq.push({f_new, v});
            }
        }
    }

    ShortestPathResult result;
    result.id = queryId;
    
    if (path_found){
        result.possible = true;
        result.minimum_distance = (mode == "distance") ? g_score[end] : -1.0;
        result.minimum_time = (mode == "time") ? g_score[end] : -1.0;
        
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