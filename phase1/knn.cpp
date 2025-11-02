#include "knn.hpp"
#include <vector>
#include <string>
#include <queue>
#include <map>

// the idea here is popping largest distances until there are k nodes left 

double SquaredDist(double lat1, double lon1, double lat2, double lon2) {
    return pow(lat1 - lat2, 2) + pow(lon1 - lon2, 2);
}
struct CompareCost {
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) {
        return a.first > b.first; 
    }
};
json findKnn(const Graph& graph, const json& query){
    int queryId = query.at("id");
    std::string targetType = query.at("type"); 
    json queryPoint = query.at("query_point");
    double qLat = queryPoint.at("lat");
    double qLon = queryPoint.at("lon");
    int k = query.at("k");
    std::string metric = query.at("metric");

    json result;
    result["id"] = queryId;
    // ai , like how to write results 
    json nodeList = json::array();

    if (metric == "euclidean"){
        std::priority_queue<std::pair<double,int>> pq;
        for (int nodeId : graph.getAllNodeIds()){
            const Node& node = graph.getNode(nodeId);

            bool typeMatch = false;
            for (const std::string& poi : node.pois) {
                if (poi == targetType) {
                    typeMatch = true;
                    break;
                }
            }
            if(typeMatch){
                double dist = SquaredDist(qLat, qLon, node.lat, node.lon);
                if (pq.size() < k){
                    pq.push({dist, nodeId});
                }
                else if(dist < pq.top().first){
                    pq.pop();
                    pq.push({dist,nodeId});
                }
            }
        }
        while (!pq.empty()) {
            nodeList.push_back(pq.top().second);
            pq.pop();
        }
        std::reverse(nodeList.begin(), nodeList.end());
        result["nodes"] = nodeList;
    }
    else if(metric == "shortest_path"){
        int start = -1;
        double min_dist = std::numeric_limits<double>::max(); //took from practice labs
        // this we are doing because , the given location may not be at node or may not be connected to road directly 
        //so we are checking the nearest neighbours , if it is the node itself , the no issue;
        for(int nodeId : graph.getAllNodeIds()){
            const Node& node = graph.getNode(nodeId);
            double dist = SquaredDist(qLat , qLon , node.lat ,node.lon);
            if(dist < min_dist){
                min_dist = dist;
                start = nodeId;
            }
        }
        if(start == -1){
            result["nodes"] = nodeList;
            return result;
        }
        //from this , it will be a normal dijkstra's with some changes
        std::priority_queue<std::pair<double,int> , std::vector<std::pair<double,int>> , CompareCost > pq;
        std::map<int,double> cost;
        std::vector<int> k_neigh;
        cost[start] = 0.0;
        pq.push({0.0,start});
        while(!pq.empty() && k_neigh.size() < k){
            double currentCost = pq.top().first;
            int currentNode = pq.top().second;
            pq.pop();
            
            if(cost.count(currentNode) && currentCost > cost[currentNode]){
                continue;
            }
            const Node& node = graph.getNode(currentNode);
            bool typeMatch = false;
            for (const std::string& poi : node.pois) {
                if (poi == targetType) {
                    typeMatch = true;
                    break;
                }
            }
            if(typeMatch){
                //since we put k_neigh.size() < k this condition on;y nearest neighbours will be stored and pq will take care of shortness;
                k_neigh.push_back(currentNode);
            }
            for (int edgeId : graph.getNeighborEdges(currentNode)){
                const Edge& edge = graph.getEdge(edgeId);
                double edgeCost = edge.length;
                int neigh = (edge.u == currentNode) ? edge.v : edge.u;

                double newCost = currentCost + edgeCost;

                if (!cost.count(neigh) || newCost < cost[neigh]) {
                    cost[neigh] = newCost;
                    pq.push({newCost, neigh});
                }
            }
        }
        for(int nodeId : k_neigh) {
            nodeList.push_back(nodeId);
        }
        result["nodes"] = nodeList;
    }
    
    return result;
}
