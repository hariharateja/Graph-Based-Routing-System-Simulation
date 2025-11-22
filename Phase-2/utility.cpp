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

