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
    for (const auto& pair : m_edges) {
        const Edge& e = pair.second;
        m_adjList[e.u].push_back(e.id);
        if (!e.oneway) {
            m_adjList[e.v].push_back(e.id);
        }
    }
}

void Graph::loadFromJson(const json& graphData) {
    m_nodes.clear();
    m_edges.clear();

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

void Graph::removeEdge(int edgeId) {
    if (m_edges.count(edgeId)) {
        m_edges.erase(edgeId);
        buildAdjacencyList(); 
    }
}

void Graph::modifyEdge(int edgeId, const json& patch) {
    if (m_edges.count(edgeId)) {
        if (patch.contains("length")) {
            m_edges.at(edgeId).length = patch.at("length");
        }
    }
}

const std::vector<int>& Graph::getNeighborEdges(int nodeId) const {
    auto it = m_adjList.find(nodeId);
    if (it == m_adjList.end()) {
        static const std::vector<int> empty_vec;
        return empty_vec;
    }
    return it->second;
}

const Edge& Graph::getEdge(int edgeId) const {
    return m_edges.at(edgeId);
}

const Node& Graph::getNode(int nodeId) const {
    return m_nodes.at(nodeId);
}

std::vector<int> Graph::getAllNodeIds() const {
    std::vector<int> ids;
    ids.reserve(m_nodes.size()); // Optimize allocation
    for (const auto& pair : m_nodes) {
        ids.push_back(pair.first);
    }
    return ids;
}
