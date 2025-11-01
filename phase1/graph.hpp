#include <string>
#include <vector>
#include<map>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

//struct for node
struct Node {
    int id;         
    double lat;
    double lon;
    std::vector<std::string> pois;

    static Node fromJson(const json& j);
};

struct Edge {
    int id;
    int u;
    int v;
    double length;
    double average_time;
    std::vector<double> speed_profile; 
    bool oneway;
    std::string road_type; 

    static Edge fromJson(const json& j);
};

class Graph{
    private:
        std::map<int,Node> m_nodes;
        std::map<int,Edge> m_edges;
        std::map<int, std::vector<int>> m_adjList;
        void buildAdjacencyList();

    public:
        void loadFromJson(const json& graphData);
        void removeEdge(int edgeId);
        void modifyEdge(int edgeId, const json& patch);

        const std::vector<int>& getNeighborEdges(int nodeId) const;
        const Edge& getEdge(int edgeId) const;
        const Node& getNode(int nodeId) const;
        std::vector<int> getAllNodeIds() const;
};