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
};
struct CompareCost {
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) const {
        return a.first > b.first;
    }
};

// using Dikstra algo with banned edges and nodes
Path dijkstra (const Graph& graph, int source, int target, const std::unordered_set<int>& bannedEdges, const std::unordered_set<int>& bannedNodes){
    Path resultpath;
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

    if( bannedNodes.count(source)) return resultpath;
    dist[source] = 0.0;
    pq.push({0.0, source});

    while(!pq.empty()){
        auto [currdist, u] = pq.top();
        pq.pop();

        if(currdist > dist[u]) continue;
        if(u == target) break;

        for( auto [v,edgeid,w] : graph.neighborsWithEdge(u)){
            if(bannedEdges.count(edgeid) || bannedNodes.count(v)) continue;
            double newdist = currdist + w;
            if(!dist.count(v) || newdist < dist[v]){
                dist[v] = newdist;
                parentNode[v] = u;
                parentEdge[v] = edgeid;
                pq.push({newdist, v});
            }
        }
    }

    if(!dist.count(target)) return resultpath;

    std::vector<int> nodeslist;
    std::vector<int> edgelist;
    int curr = target;
    while(curr != source){
        nodeslist.push_back(curr);
        edgelist.push_back( parentEdge[curr]);
        curr = parentNode[curr];
    }
    nodeslist.push_back(source);
    std::reverse(nodeslist.begin(), nodeslist.end());
    std::reverse(edgelist.begin(), edgelist.end());

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
    std::string type = query.at("type");
    int id = query.at("id");
    int source = query.at("source");
    int target = query.at("target");
    int K = query.at("k");

    json out;
    out["type"] = type;
    out["id"] = id;
    out["paths"] = json::array();
    if (K <= 0) return out;

    std::unordered_set<int> emptyEdges, emptyNodes;

    Path first = dijkstra(graph, source, target, emptyEdges, emptyNodes);

    if (first.nodes.empty()) {
        return out;
    }

    std::vector<Path> shortestPaths;
    shortestPaths.push_back(first);
    std::priority_queue<Path, std::vector<Path>, ComparePath> candidatePaths;
    std::set<std::string> seenPaths;

    for(int k = 1; k < K; ++k) {
        const Path& prevPath = shortestPaths[k - 1];

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
            }

            for (size_t j = 0; j < i  ; ++j) {
                bannedNodes.insert(prevPath.nodes[j]);
            }

            Path spurPath = dijkstra(graph, spurNode, target, bannedEdges, bannedNodes);

            if (spurPath.nodes.empty()) continue;

            Path candidatePath;
            candidatePath.nodes = rootnodes;
            candidatePath.nodes.insert(candidatePath.nodes.end(), spurPath.nodes.begin() + 1, spurPath.nodes.end());
            candidatePath.edgeIds = rootedges;
            candidatePath.edgeIds.insert(candidatePath.edgeIds.end(), spurPath.edgeIds.begin(), spurPath.edgeIds.end());

            double rootcost = 0.0;
            for( int edgeid : rootedges){
                rootcost += graph.edgeWeight(edgeid);
            }
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