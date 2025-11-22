 #include "utility.hpp"

static const double R = 6371000.0;

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

double toRad(double deg) {
    return deg * M_PI / 180.0;
}

double heuristic(const Graph& graph, int node, int target){
    const Node &n1 = graph.getNode(node);
    const Node &n2 = graph.getNode(target);
    
    double lat1 = toRad(n1.lat);
    double lon1 = toRad(n1.lon);
    double lat2 = toRad(n2.lat);
    double lon2 = toRad(n2.lon);

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;
    
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double d = R * c;
    return d;
}

