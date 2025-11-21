#include "graph.hpp"
#include <iostream> 

Node Node::fromJson(const json& j) {
    Node n;
    n.id = j.at("id");
    n.lat = j.at("lat");
    n.lon = j.at("lon");
    if (j.contains("pois")) {
        n.pois = j.at("pois").get<std::vector<std::string>>();
    }
    return n;
}

Edge Edge::fromJson(const json& j) {
    Edge e;
    e.id = j.at("id");
    e.u = j.at("u");
    e.v = j.at("v");
    e.length = j.at("length");
    e.average_time = j.at("average_time");
    e.oneway = j.at("oneway");
    e.road_type = j.at("road_type");
    if (j.contains("speed_profile")) {
        e.speed_profile = j.at("speed_profile").get<std::vector<double>>();
    }
    return e;
}

void Graph::buildAdjacencyList() {
    m_adjList.clear();

    m_adjList.assign(m_nodes.size(),{});
    for (const auto& pair : m_edges) {
        const Edge& e = pair.second;
        m_adjList[e.u].push_back(e.id);
        if (!e.oneway) m_adjList[e.v].push_back(e.id);
    }
}

void Graph::loadFromJson(const json& graphData) {
    m_nodes.clear();
    m_edges.clear();

    m_nodes.resize(graphData.at("nodes").size());
    for (const auto& nodeJson : graphData.at("nodes")) {
        Node n = Node::fromJson(nodeJson);
        m_nodes[n.id] = n;
    }
    for (const auto& edgeJson : graphData.at("edges")) {
        Edge e = Edge::fromJson(edgeJson);
        m_edges[e.id] = e;
    }
    
    std::cout << "Graph loaded: " << m_nodes.size() << " nodes, " 
              << m_edges.size() << " edges." << std::endl;
              
    buildAdjacencyList();
}

bool Graph::removeEdge(int edgeId) {
    if (m_edges.count(edgeId)) {
        Edge& e = m_edges[edgeId];
        auto& vec1 = m_adjList[e.u];
        auto itr = std::remove(vec1.begin(),vec1.end(),e.id);

        if(itr == vec1.end()) return false; // already removed edge

        vec1.erase(itr,vec1.end());
        if(!e.oneway){
            auto& vec2 = m_adjList[e.v];
            vec2.erase(std::remove(vec2.begin(),vec2.end(),e.id),vec2.end());
        }
        return true;
    }
    else {
        return false;
    }
}

bool Graph::modifyEdge(const json& event) {
    int edgeId = event.at("edge_id");
    if (m_edges.count(edgeId)) {
        Edge& e = m_edges[edgeId];
        auto& vec = m_adjList[e.u];

        bool disabled = false;
        if (std::find(vec.begin(),vec.end(),e.id) == vec.end()) { // if the edge was disabled
            vec.push_back(e.id);
            if(!e.oneway) m_adjList[e.v].push_back(e.id);
            disabled = true;
        }
        
        // if no patch nor enabling an edge
        if(!disabled && (!event.contains("patch") || event.at("patch").empty())){
            return false;
        }
        
        // if there is patch
        const json patch = event.at("patch");

        if (patch.contains("length")) e.length = patch.at("length");
        if (patch.contains("average_time")) e.average_time = patch.at("average_time");
        if (patch.contains("speed_profile")) e.speed_profile = patch.at("speed_profile").get<std::vector<double>>();
        
        return true;
    }
    else {
        return false;
    }
}

const std::vector<int>& Graph::getNeighborEdges(int nodeId) const {
    static const std::vector<int> dummy = {};
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) return dummy;
    return m_adjList[nodeId];
    }

const Edge& Graph::getEdge(int edgeId) const {
    return m_edges.at(edgeId);
}

const Node& Graph::getNode(int nodeId) const {
    if(nodeId < m_nodes.size()) return m_nodes[nodeId];
    else{
        std::cout << "Invalid ID: " << nodeId << std::endl;
        std::cout << "Graph not loaded!, please load first" << std::endl;
        static const Node dummy = Node::fromJson(json{{"id",-1},{"lat",0.0},{"lon",0.0}});
        return dummy; // dummy
    }
}

std::vector<int> Graph::getAllNodeIds() const {
    std::vector<int> ids;
    ids.reserve(m_nodes.size()); // Optimize allocation
    for (const auto& node : m_nodes) {
        ids.push_back(node.id);
    }
    return ids;
}


double Graph::edgeWeight(int edgeId) const {
    auto it = m_edges.find(edgeId);
    if (it == m_edges.end()) return std::numeric_limits<double>::infinity();
    return it->second.length;
}

std::vector<std::tuple<int,int,double>> Graph::neighborsWithEdge(int nodeId) const {
    std::vector<std::tuple<int,int,double>> out;
    if (nodeId < 0 || nodeId >= (int)m_adjList.size()) return out;
    for (int eid : m_adjList[nodeId]) {
        auto it = m_edges.find(eid);
        if (it == m_edges.end()) continue;
        const Edge& e = it->second;
        int v = (e.u == nodeId) ? e.v : e.u;
        out.emplace_back(v, eid, e.length);
    }
    return out;
}