#include "knn.hpp"
#include <vector>
#include <string>
#include <queue>

// the idea here is popping largest distances until there are k nodes left 

double SquaredDist(double lat1, double lon1, double lat2, double lon2) {
    return pow(lat1 - lat2, 2) + pow(lon1 - lon2, 2);
}

json findKnn(const Graph& graph, const json& query){
    int queryId = query.at("id");
    std::string targetType = query.at("type"); 
    json queryPoint = query.at("query_point");
    double qLat = queryPoint.at("lat");
    double qLon = queryPoint.at("lon");
    int k = query.at("k");
    std::string metric = query.at("metric");
}
