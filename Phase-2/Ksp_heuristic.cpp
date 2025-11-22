#include "Ksp_heuristic.hpp"
#include <vector>
#include <queue>
#include <set>
#include <unordered_set>
#include <map>
#include <limits>
#include <algorithm>
#include <sstream>

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

// A* algorithm regular
Path A_star_regular(const Graph& graph, int source, int target) {
    Path resultpath;
    if (source == target) {
        resultpath.nodes = {source};
        resultpath.cost = 0.0;
        return resultpath;
    }

    std::unordered_map<int, double> dist;
    std::unordered_map<int, int> parentNode;
    std::unordered_map<int, int> parentEdge;

    std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, CompareCost> pq;

    dist[source] = 0.0;
    pq.push({heuristic(graph, source, target), source});

    while (!pq.empty()) {
        auto [currdist, u] = pq.top();
        pq.pop();

        if (u == target) break;
        if (currdist - heuristic(graph, u, target) > dist[u]) continue;

        for (auto [v, edgeid, w] : graph.neighborsWithEdge(u)) {
            if (!dist.count(v) || dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                parentNode[v] = u;
                parentEdge[v] = edgeid;
                pq.push({dist[v] + heuristic(graph, v, target), v});
            }
        }
    }

    if (!dist.count(target)) return resultpath;

    // Backtrack
    int curr = target;
    while (curr != source) {
        resultpath.nodes.push_back(curr);
        resultpath.edgeIds.push_back(parentEdge[curr]);
        curr = parentNode[curr];
    }
    resultpath.nodes.push_back(source);
    //reverse to get correct order
    std::reverse(resultpath.nodes.begin(), resultpath.nodes.end());
    std::reverse(resultpath.edgeIds.begin(), resultpath.edgeIds.end());
    resultpath.cost = dist[target];
    return resultpath;
}

//A-star with banned edges and nodes
static Path A_star_with_bans(const Graph& graph, int source, int target, const std::unordered_set<int>& bannedEdges, const std::unordered_set<int>& bannedNodes)
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

//ESX-MinW helpers

static double computeOverlapPercent(const Path& a, const Path& b) {
    if (a.edgeIds.empty() || b.edgeIds.empty()) return 0.0;

    std::unordered_set<int> edges(b.edgeIds.begin(), b.edgeIds.end());
    int common = 0;
    for (int e : a.edgeIds) {
        if (edges.count(e)) ++common;
    }

    double denom = b.edgeIds.size();
    if (denom <= 0.0) return 0.0;
    return 100.0 * static_cast<double>(common) / denom;
}

// compute Total Penalty for a set of paths given their indices
static double computeTotalPenaltyForSet(const std::vector<Path>& allPaths, const std::vector<int>& indices, double baseCost, double overlapThreshold) {
    int k = static_cast<int>(indices.size());
    if (k <= 0) return 0.0;

    double total = 0.0;
    for (int i = 0; i < k; ++i) {
        int iIdx = indices[i];
        const Path& pi = allPaths[iIdx];

        // Distance penalty:
        // Distance Penalty for i = percentage difference from shortest / 100 + 0.1
        // percentage difference = (len_i - len_shortest) / len_shortest * 100
        double distPenalty = 0.1;
        if (baseCost > 1e-9) {
            double percentDiff = (pi.cost - baseCost) / baseCost * 100.0;
            if (percentDiff < 0.0) percentDiff = 0.0; // safety
            double frac = percentDiff / 100.0;
            distPenalty = frac + 0.1;
        }

        // Overlap penalty:
        // number of other paths with overlap% > overlapThreshold
        int overlapPenalty = 0;
        for (int j = 0; j < k; ++j) {
            int jIdx = indices[j];
            double ov = computeOverlapPercent(pi, allPaths[jIdx]);
            if (ov > overlapThreshold) {
                ++overlapPenalty;
            }
        }

        total += static_cast<double>(overlapPenalty) * distPenalty;
    }

    return total;
}

static std::vector<Path> YensCandidates(const Graph& graph, int source , int target , int maxPaths) {
    std::vector<Path> Paths;
    if (maxPaths <= 0) return Paths;

    Path first = A_star_regular(graph, source, target);
    if (first.nodes.empty()) {
        return Paths;
    }
    Paths.push_back(first);

    std::priority_queue<Path, std::vector<Path>, ComparePath> candidatePaths;
    std::set<std::string> seenPaths;
    seenPaths.insert(makePathSignature(first));

    for(int k=1; k<maxPaths; ++k) {
        const Path& prevPath = Paths[k - 1];

        int len = static_cast<int>(prevPath.nodes.size());
        if( len < 2) break;

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

            for (const Path& p : Paths) {
                if (p.nodes.size() > i + 1  && std::equal(rootnodes.begin(), rootnodes.end(), p.nodes.begin())) {
                    bannedEdges.insert(p.edgeIds[i]);
                }
            } //create banned edges
 
            for (size_t j = 0; j < i  ; ++j) {
                bannedNodes.insert(prevPath.nodes[j]);
            } //create banned nodes

            //use A* to find spur path
            Path spurPath = A_star_with_bans(graph, spurNode, target, bannedEdges, bannedNodes);
            
            if (spurPath.nodes.empty()) continue;

            Path candidatePath;
            //combine root and spur paths nodes
            candidatePath.nodes = rootnodes;
            candidatePath.nodes.insert(candidatePath.nodes.end(), spurPath.nodes.begin() + 1, spurPath.nodes.end());
            //combine root and spur paths edges
            candidatePath.edgeIds = rootedges;
            candidatePath.edgeIds.insert(candidatePath.edgeIds.end(), spurPath.edgeIds.begin(), spurPath.edgeIds.end());

            
            candidatePath.cost = prefixCost[i] + spurPath.cost;

            std::string signature = makePathSignature(candidatePath);
            if (seenPaths.insert(signature).second) {
                candidatePaths.push(candidatePath);
            }
        }

        if (candidatePaths.empty()) break;

        Path bestCandidate = candidatePaths.top();
        candidatePaths.pop();
        Paths.push_back(bestCandidate);
    }
    return Paths;
}

json findKsp_heuristic(const Graph& graph, const json& query) {
    // ESX-MinW applied Yens algorithm to find K shortest paths
    std::string type = query.at("type");
    int id = query.at("id");
    int source = query.at("source");
    int target = query.at("target");
    int K = query.at("k");
    int overlap_threshold = query.value("overlap_threshold", 0); // default overlap is 0

    // output json
    json out; 
    out["id"] = id;
    out["paths"] = json::array();
    if (K <= 0) return out;

    int maxCandidatePaths = 3 * K;
    maxCandidatePaths = std::min(maxCandidatePaths, 50); // at most 50 candidates

    std::vector<Path> candidatePaths = YensCandidates(graph, source, target, maxCandidatePaths);

    if (candidatePaths.empty()) {
        return out;
    }

    double baseCost = candidatePaths[0].cost;

    int availablePaths = static_cast<int>(candidatePaths.size());
    int pathsToSelect = std::min(K, availablePaths);

    std::vector<int> selectedIndices = {0}; // always select the shortest path

    for(int i=1; i < pathsToSelect; i++){
        double bestPenalty = std::numeric_limits<double>::max();    
        int bestIndex = -1;

        for(int j=1; j < availablePaths; j++){
            if (std::find(selectedIndices.begin(), selectedIndices.end(), j) != selectedIndices.end()) {
                continue; // already selected
            }

            std::vector<int> tempIndices = selectedIndices;
            tempIndices.push_back(j);

            double penalty = computeTotalPenaltyForSet(candidatePaths, tempIndices, baseCost, static_cast<double>(overlap_threshold));

            if (penalty < bestPenalty || (penalty == bestPenalty && (bestIndex == -1 || candidatePaths[j].cost < candidatePaths[bestIndex].cost))){
                bestPenalty = penalty;
                bestIndex = j;
            }
        }

        if (bestIndex == -1) {
            break; // no more paths can be selected
        } else {
            selectedIndices.push_back(bestIndex);
        }
    }

    // Prepare output
    std::vector<Path> finalPaths;
    for(int idx : selectedIndices) {
        finalPaths.push_back(candidatePaths[idx]);
    }

    std::sort(finalPaths.begin(), finalPaths.end(), [](const Path& a, const Path& b) {
        return a.cost < b.cost;
    });

    for(const Path& p : finalPaths) {
        json item;
        item["path"] = p.nodes;
        item["length"] = std::round(p.cost * 1000.0)/1000.0;
        out["paths"].push_back(item);
    }

    return out;
}
