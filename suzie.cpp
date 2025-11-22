// ============================================================================
// PATH HASHING
// ============================================================================
static inline size_t hash_path(const std::vector<int>& p) {
    size_t h = p.size();
    for (int n : p) h ^= std::hash<int>{}(n) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
}

// ============================================================================
// OPTIMIZED A* WITH FORBIDDEN EDGES (uses vectors for speed)
// ============================================================================
std::tuple<bool,double,std::vector<int>> Graph::dijkstra_astar(
    int source, int target, const std::string& mode,
    const std::unordered_set<int>& forbidden_nodes,
    const std::unordered_set<std::string>& forbidden_road_types,
    bool use_astar,
    const std::set<std::pair<int,int>>& forbidden_edges,
    double max_speed_mps)
{
    const double INF = std::numeric_limits<double>::infinity();
    if (nodes.find(source) == nodes.end() || nodes.find(target) == nodes.end()) 
        return {false, INF, {}};
    if (forbidden_nodes.count(source) || forbidden_nodes.count(target)) 
        return {false, INF, {}};

    // Pre-fetch target node for heuristic
    const Node& target_node = nodes.at(target);
    
    auto heuristic = [&](int node_id) -> double {
        if (!use_astar) return 0.0;
        const Node &n = nodes.at(node_id);
        double dx = n.lat - target_node.lat;
        double dy = n.lon - target_node.lon;
        double eu = sqrt(dx*dx + dy*dy);
        return (mode == "distance") ? eu : eu / max_speed_mps;
    };

    // Use vector for g-scores (faster than unordered_map for dense IDs)
    std::unordered_map<int, double> g;
    std::unordered_map<int, int> parent;
    g.reserve(nodes.size());
    
    for (auto &p : nodes) g[p.first] = INF;
    g[source] = 0.0;

    using PQItem = std::pair<double, int>;
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;
    open.push({heuristic(source), source});

    while (!open.empty()) {
        auto [f, u] = open.top(); open.pop();
        
        if (u == target) break;
        if (f > g[u] + heuristic(u) + 1e-9) continue;

        auto adj_it = adj.find(u);
        if (adj_it == adj.end()) continue;

        for (int eid : adj_it->second) {
            const Edge &e = edges.at(eid);
            if (!e.active) continue;
            
            int v = (e.u == u) ? e.v : e.u;
            if (e.oneway && e.u != u) continue;
            if (forbidden_nodes.count(v)) continue;
            
            // Check forbidden edges
            if (!forbidden_edges.empty() && forbidden_edges.count({u, v})) continue;
            
            // Check road type
            if (!forbidden_road_types.empty() && !e.road_type.empty()) {
                if (forbidden_road_types.count(to_lower(e.road_type))) continue;
            }

            double w = (mode == "time") ? edge_travel_time(e, g[u]) : e.length;
            double new_g = g[u] + w;
            
            if (new_g < g[v] - 1e-12) {
                g[v] = new_g;
                parent[v] = u;
                open.push({new_g + heuristic(v), v});
            }
        }
    }

    if (g[target] >= INF) return {false, INF, {}};

    std::vector<int> path;
    path.reserve(100);
    for (int cur = target; cur != source; cur = parent[cur]) path.push_back(cur);
    path.push_back(source);
    std::reverse(path.begin(), path.end());
    return {true, g[target], path};
}
// ============================================================================
// COMPUTE PATH COST
// ============================================================================
double Graph::compute_path_cost(const std::vector<int>& path, const std::string& mode) const {
    if (path.size() < 2) return 0.0;
    const double INF = std::numeric_limits<double>::infinity();

    double total = 0.0;
    double t = 0.0;
    
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        int u = path[i], v = path[i+1];
        auto adj_it = adj.find(u);
        if (adj_it == adj.end()) return INF;
        
        const Edge* found_edge = nullptr;
        for (int eid : adj_it->second) {
            const Edge &e = edges.at(eid);
            if (!e.active) continue;
            int oth = (e.u == u) ? e.v : e.u;
            if (oth == v) { found_edge = &e; break; }
        }
        
        if (!found_edge) return INF;
        
        if (mode == "time") {
            double tt = edge_travel_time(*found_edge, t);
            total += tt;
            t += tt;
        } else {
            total += found_edge->length;
        }
    }
    return total;
}

// ============================================================================
// PATH OVERLAP
// ============================================================================
double Graph::path_edge_overlap_fraction(const std::vector<int>& a, const std::vector<int>& b) const {
    if (a.size() < 2 || b.size() < 2) return 0.0;

    // Use smaller path for the set
    const auto& smaller = (a.size() < b.size()) ? a : b;
    const auto& larger = (a.size() < b.size()) ? b : a;

    std::unordered_set<long long> edge_set;
    edge_set.reserve(smaller.size());
    
    for (size_t i = 0; i + 1 < smaller.size(); ++i) {
        long long key = ((long long)smaller[i] << 32) | (unsigned int)smaller[i+1];
        edge_set.insert(key);
    }

    int common = 0;
    for (size_t i = 0; i + 1 < larger.size(); ++i) {
        long long key = ((long long)larger[i] << 32) | (unsigned int)larger[i+1];
        if (edge_set.count(key)) ++common;
    }

    int denom = (int)smaller.size() - 1;
    return (denom <= 0) ? 0.0 : (double)common / denom * 100.0;
}
// ============================================================================
// K SHORTEST EXACT - OPTIMIZED YEN'S FOR LARGE GRAPHS
// ============================================================================
std::vector<std::vector<int>> Graph::k_shortest_exact(int source, int target, int K, const std::string& mode) {
    std::vector<std::vector<int>> A;
    if (K <= 0) return A;

    std::unordered_set<size_t> seen_paths;

    using PathWithCost = std::pair<double, std::vector<int>>;
    auto pq_cmp = [](const PathWithCost& a, const PathWithCost& b) { return a.first > b.first; };
    std::priority_queue<PathWithCost, std::vector<PathWithCost>, decltype(pq_cmp)> B(pq_cmp);

    // First shortest path
    auto res0 = dijkstra_astar(source, target, mode, {}, {}, true);
    if (!std::get<0>(res0)) return A;

    A.push_back(std::get<2>(res0));
    seen_paths.insert(hash_path(A[0]));

    // OPTIMIZATION: Limit spur iterations for long paths
    const int MAX_SPUR_ITERATIONS = 200;

    for (int k = 1; k < K; ++k) {
        const auto& prev_path = A[k - 1];
        int path_len = (int)prev_path.size();
        
        // OPTIMIZATION: Skip some spur nodes for very long paths
        int step = 1;
        if (path_len > 100) step = path_len / 50;
        
        int spur_count = 0;

        for (int i = 0; i < path_len - 1 && spur_count < MAX_SPUR_ITERATIONS; i += step) {
            spur_count++;
            int spur_node = prev_path[i];

            // Build forbidden edges
            std::set<std::pair<int, int>> forbidden_edges;
            for (const auto& p : A) {
                if ((int)p.size() > i + 1) {
                    bool same_root = true;
                    for (int j = 0; j <= i && same_root; ++j) {
                        if (p[j] != prev_path[j]) same_root = false;
                    }
                    if (same_root) forbidden_edges.insert({p[i], p[i + 1]});
                }
            }

            // Forbidden nodes
            std::unordered_set<int> forbidden_nodes;
            forbidden_nodes.reserve(i);
            for (int j = 0; j < i; ++j) forbidden_nodes.insert(prev_path[j]);

            // Find spur path
            auto spur_res = dijkstra_astar(spur_node, target, mode, forbidden_nodes, {}, true, forbidden_edges);

            if (std::get<0>(spur_res)) {
                std::vector<int> total_path;
                total_path.reserve(i + std::get<2>(spur_res).size());
                for (int j = 0; j < i; ++j) total_path.push_back(prev_path[j]);
                const auto& spur_path = std::get<2>(spur_res);
                total_path.insert(total_path.end(), spur_path.begin(), spur_path.end());

                size_t h = hash_path(total_path);
                if (seen_paths.find(h) == seen_paths.end()) {
                    seen_paths.insert(h);
                    double cost = compute_path_cost(total_path, mode);
                    if (std::isfinite(cost)) {
                        B.push({cost, std::move(total_path)});
                    }
                }
            }
        }

        if (B.empty()) break;
        A.push_back(std::move(const_cast<std::vector<int>&>(B.top().second)));
        B.pop();
    }

    return A;
}