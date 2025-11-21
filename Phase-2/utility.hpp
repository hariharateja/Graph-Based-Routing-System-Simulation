#pragma once
#include <vector>
#include "graph.hpp"
struct Path {
    std::vector<int> nodes;  
    std::vector<int> edgeIds;
    double cost = 0.0;
};//to identify a path

double heuristic(const Graph& graph, int node, int target);//heuristic function for A* - Euclidean distance

std::string makePathSignature(const Path& p); //to identify unique paths [N:1,4,7|E:10,12]