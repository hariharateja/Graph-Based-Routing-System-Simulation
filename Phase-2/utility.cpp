 #include "utility.hpp"

//to identify unique paths [N:1,4,7|E:10,12]
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
    
    double dx = n1.lat - n2.lat;
    double dy = n1.lon - n2.lon;
    return std::sqrt(dx * dx + dy * dy);
}

