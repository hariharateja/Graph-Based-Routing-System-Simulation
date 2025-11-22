 #include "utility.hpp"

//to identify unique paths Ex: [N:1,4,7|E:10,12]
std::string makePathSignature(const Path& p) {
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

double heuristic(const Graph& graph, int node, int target){
    const Node &n1 = graph.getNode(node);
    const Node &n2 = graph.getNode(target);
    
    // double dx = n1.lat - n2.lat;
    // double dy = n1.lon - n2.lon;
    // return std::sqrt(dx * dx + dy * dy);

    const double pi = 3.141593;
    const double R = 6371000.0; // in meters
    // convert degrees to radians
    double lat1 =  (n1.lat) * (pi/180);
    double lat2 =  (n2.lat) * (pi/180);
    double lon1 =  (n1.lon) * (pi/180);
    double lon2 =  (n2.lon) * (pi/180);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a = std::sin(dlat / 2.0) * std::sin(dlat / 2.0) +
               (std::cos(lat1) * std::cos(lat2) *
               std::sin(dlon / 2.0) * std::sin(dlon / 2.0));

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

    return R * c; // in meters
}

Path A_star_with_bans(const Graph& graph, int source, int target, const std::unordered_set<int>& bannedEdges, const std::unordered_set<int>& bannedNodes)
{
    Path resultpath;
    // trivial case
    if (source == target) {
        resultpath.nodes = {source};
        resultpath.edgeIds = {};
        resultpath.cost = 0.0;
        return resultpath;
    }

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int>  parentNode;
    std::unordered_map<int, int>  parentEdge;

    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, CompareCost> pq;

    if (bannedNodes.count(source)) return resultpath; // no path

    dist[source] = 0.0;
    pq.push({heuristic(graph, source, target), source});

    while (!pq.empty()) {
        auto [currdist, u] = pq.top();
        pq.pop();

        if (u == target) break;

        if (currdist - heuristic(graph, u, target) > dist[u]) continue;

        for (auto [v, edgeid, w] : graph.neighborsWithEdge(u)) {
            if (bannedEdges.count(edgeid) || bannedNodes.count(v)) continue;

            double newDist = dist[u] + w;
            if (!dist.count(v) || newDist < dist[v]) {
                dist[v] = newDist;
                parentNode[v]= u;
                parentEdge[v]= edgeid;
                pq.push({newDist + heuristic(graph, v, target), v});
            }
        }
    }

    if (dist.find(target) == dist.end()) return resultpath; // no path

    // backtrack
    std::vector<int> nodeslist;
    std::vector<int> edgelist;

    int curr = target;
    while (curr != source) {
        nodeslist.push_back(curr);
        edgelist.push_back(parentEdge[curr]);
        curr = parentNode[curr];
    }
    nodeslist.push_back(source);

    std::reverse(nodeslist.begin(), nodeslist.end());
    std::reverse(edgelist.begin(), edgelist.end());

    resultpath.nodes   = std::move(nodeslist);
    resultpath.edgeIds = std::move(edgelist);
    resultpath.cost    = dist[target];
    return resultpath;
}
