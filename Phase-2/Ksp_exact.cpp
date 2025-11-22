#include "Ksp_exact.hpp"
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include <sstream>
#include <string>


// Yens algorithm
json findKsp_exact(const Graph& graph, const json& query) {
    // parse query
    int id = query.at("id");
    int source = query.at("source");
    int target = query.at("target");
    int K = query.at("k");
    // output json
    json out;
    out["id"] = id;
    out["paths"] = json::array();
    if (K <= 0) return out;

    std::vector<double> h = heuristic_values(graph, target);

    Path first = A_star_with_bans(graph, source, target, {} , {}, h);

    if (first.nodes.empty()) {
        return out;
    }

    std::vector<Path> shortestPaths;//shortest paths
    shortestPaths.reserve(K);
    shortestPaths.push_back(first);

    std::priority_queue<Path, std::vector<Path>, ComparePath> candidatePaths;
    std::set<std::string> seenPaths;
    std::string firstSignature = makePathSignature(first);
    seenPaths.insert(firstSignature);    

    for(int k = 1; k < K; ++k) { // loop over all K 
        const Path& prevPath = shortestPaths[k - 1];

        int len = static_cast<int>(prevPath.nodes.size());
        if( len < 2) break;

        // Compute prefix costs of the nodes in the previous path
        std::vector<double> prefixCost(prevPath.nodes.size(), 0.0);
        for (size_t j = 0; j < prevPath.edgeIds.size(); ++j) {
            prefixCost[j + 1] = prefixCost[j] + graph.edgeWeight(prevPath.edgeIds[j]);
        }

        for (size_t i = 0; i < prevPath.nodes.size() - 1; ++i) {
            int spurNode = prevPath.nodes[i];
            std::vector<int> rootedges(prevPath.edgeIds.begin(), prevPath.edgeIds.begin() + i);
            std::vector<int> rootnodes(prevPath.nodes.begin(), prevPath.nodes.begin() + i + 1);

            std::unordered_set<int> bannedEdges;
            std::unordered_set<int> bannedNodes(rootnodes.begin(), rootnodes.end() - 1); // create banned nodes

            for (const Path& p : shortestPaths) {
                if (p.nodes.size() > i + 1 && std::equal(rootnodes.begin(), rootnodes.end(), p.nodes.begin())) { // Prefix match
                    bannedEdges.insert(p.edgeIds[i]);
                }
            } //create banned edges

            //use A* to find spur path
            Path spurPath = A_star_with_bans(graph, spurNode, target, bannedEdges, bannedNodes, h);
            
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
            if (seenPaths.insert(signature).second) { // checks whether the insert is success/not
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
            {"length", std::round(p.cost * 1000.0)/1000.0 }
        });
    }

    return out;
}