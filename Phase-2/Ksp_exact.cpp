#include "Ksp_exact.hpp"
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>


struct ComparePath {
    bool operator()(const Path& a, const Path& b) const {
        return a.cost > b.cost;
    }
}; // compare path based on cost
struct CompareCost {
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) const {
        return a.first > b.first;
    }
}; // compare cost for pq 

// using Dikstra algo with banned edges and nodes
Path dijkstra (const Graph& graph, int source, int target, const std::unordered_set<int>& bannedEdges, const std::unordered_set<int>& bannedNodes){
    Path resultpath;
    //trivial case
    if(source == target){
        resultpath.nodes = {source};
        resultpath.edgeIds = {};
        resultpath.cost = 0.0;
        return resultpath;
    }

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parentNode;
    std::unordered_map<int, int> parentEdge;

    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, CompareCost> pq;

    if( bannedNodes.count(source)) return resultpath;// no path
    dist[source] = 0.0;
    pq.push({0.0, source});

    while(!pq.empty()){
        auto [currdist, u] = pq.top();
        pq.pop();

        if(currdist > dist[u]) continue;
        if(u == target) break;

        for( auto [v,edgeid,w] : graph.neighborsWithEdge(u)){
            if(bannedEdges.count(edgeid) || bannedNodes.count(v)) continue;
            
            if(!dist.count(v) || currdist + w < dist[v]){
                dist[v] = currdist + w;
                parentNode[v] = u;
                parentEdge[v] = edgeid;
                pq.push({currdist + w, v});
            }
        }
    }

    if(dist.find(target) == dist.end()) return resultpath;

    std::vector<int> nodeslist;
    std::vector<int> edgelist;
    // backtrack
    int curr = target; 
    while(curr != source){
        nodeslist.push_back(curr);
        edgelist.push_back( parentEdge[curr]);
        curr = parentNode[curr];
    }
    nodeslist.push_back(source);

    //reverse to get correct order
    std::reverse(nodeslist.begin(), nodeslist.end());
    std::reverse(edgelist.begin(), edgelist.end());
    //store in resultpath
    resultpath.nodes = nodeslist;
    resultpath.edgeIds = edgelist;
    resultpath.cost = dist[target];
    return resultpath;
}


//to identify unique paths [N:1,4,7|E:10,12]
static std::string makePathSignature(const Path& p) {
    std::ostringstream oss;
    oss << "[N:";
    for (size_t i = 0; i < p.nodes.size(); ++i) {
        oss << p.nodes[i];
        if (i + 1 < p.nodes.size()) oss << ',';  // commas between nodes
    }
    oss << "|E:";
    for (size_t i = 0; i < p.edgeIds.size(); ++i) {
        oss << p.edgeIds[i];
        if (i + 1 < p.edgeIds.size()) oss << ','; // commas between edges
    }
    oss << "]";
    return oss.str();
}

//yens algorithm
json findKsp(const Graph& graph, const json& query) {
    // parse query
    std::string type = query.at("type");
    int id = query.at("id");
    int source = query.at("source");
    int target = query.at("target");
    int K = query.at("k");
    // output json
    json out;
    out["type"] = type;
    out["id"] = id;
    out["paths"] = json::array();
    if (K <= 0) return out;

    Path first = dijkstra(graph, source, target, {} , {});

    if (first.nodes.empty()) {
        return out;
    }

    std::vector<Path> shortestPaths;//shortest paths
    shortestPaths.push_back(first);

    std::priority_queue<Path, std::vector<Path>, ComparePath> candidatePaths;
    std::set<std::string> seenPaths;

    for(int k = 1; k < K; ++k) {//loop over all K 
        const Path& prevPath = shortestPaths[k - 1];

        // Compute prefix costs for the previous path
        std::vector<double> prefixCost(prevPath.edgeIds.size() + 1, 0.0);
        for (size_t j = 0; j < prevPath.edgeIds.size(); ++j) {
            prefixCost[j + 1] = prefixCost[j] + graph.edgeWeight(prevPath.edgeIds[j]);
        }

        for (size_t i = 0; i < prevPath.nodes.size() - 1; ++i) {
            int spurNode = prevPath.nodes[i];
            std::vector<int> rootedges(prevPath.edgeIds.begin(), prevPath.edgeIds.begin() + i);
            std::vector<int> rootnodes(prevPath.nodes.begin(), prevPath.nodes.begin() + i + 1);

            std::unordered_set<int> bannedEdges;
            std::unordered_set<int> bannedNodes;

            for (const Path& p : shortestPaths) {
                if (p.nodes.size() > i + 1 && std::equal(rootnodes.begin(), rootnodes.end(), p.nodes.begin())) {
                    bannedEdges.insert(p.edgeIds[i]);
                }
            } //create banned edges
 
            for (size_t j = 0; j < i  ; ++j) {
                bannedNodes.insert(prevPath.nodes[j]);
            } //create banned nodes

            //use dijkstra to find spur path
            Path spurPath = dijkstra(graph, spurNode, target, bannedEdges, bannedNodes);
            
            if (spurPath.nodes.empty()) continue;

            Path candidatePath;
            //combine root and spur paths nodes
            candidatePath.nodes = rootnodes;
            candidatePath.nodes.insert(candidatePath.nodes.end(), spurPath.nodes.begin() + 1, spurPath.nodes.end());
            //combine root and spur paths edges
            candidatePath.edgeIds = rootedges;
            candidatePath.edgeIds.insert(candidatePath.edgeIds.end(), spurPath.edgeIds.begin(), spurPath.edgeIds.end());

            double rootcost = prefixCost[i];
            candidatePath.cost = rootcost + spurPath.cost;

            std::string signature = makePathSignature(candidatePath);
            if (seenPaths.insert(signature).second) {
                candidatePaths.push(candidatePath);
            }

        }
        if (candidatePaths.empty()) break;
        shortestPaths.push_back(candidatePaths.top());
        candidatePaths.pop();
    }

    for (const auto& p : shortestPaths) {
        out["paths"].push_back({
            {"path", p.nodes},
            {"length", p.cost}
        });
    }

    return out;
}