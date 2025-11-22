#pragma once
#include <vector>
#include "graph.hpp"
#include <cmath>
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include <sstream>
#include <string>

struct Path {
    std::vector<int> nodes;  
    std::vector<int> edgeIds;
    double cost = 0.0;
};//to identify a path

struct CompareCost {
    bool operator()(const std::pair<double,int>& a, const std::pair<double,int>& b) const {
        return a.first > b.first;
    }
}; // compare cost for pq 

struct ComparePath {
    bool operator()(const Path& a, const Path& b) const {
        return a.cost > b.cost;
    }
}; // compare path based on cost

std::string makePathSignature(const Path& p); //to identify unique paths Ex: [N:1,4,7|E:10,12]

double heuristic(const Graph& graph, int node, int target);//heuristic function for A* - Euclidean distance

std::vector<double> heuristic_values(const Graph& graph, int target);

Path A_star_with_bans(const Graph& graph, int source, int target, const std::unordered_set<int>& bannedEdges, const std::unordered_set<int>& bannedNodes,  const std::vector<double>& h);
// A* with banned edges and nodes