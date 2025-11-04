#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "graph.hpp"
#include "shortest_path.hpp"
#include "knn.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include "graph.hpp"
#include "shortest_path.hpp"
#include "knn.hpp"

using json = nlohmann::json;

Graph graph;
json processEvent(const json& event){
    std::string eventType = event.at("type");
    if (eventType == "shortest_path") {
        ShortestPathResult result = findShortestPath(graph, event);
        return result.toJson(event.at("mode"));
    }
    else if (eventType == "knn") {
        return findKnn(graph, event);

    }
    else if (eventType == "remove_edge") {
        graph.removeEdge(event.at("edge_id"));
        return json{{"done", true}};

    }
    else if (eventType == "modify_edge") {
        graph.modifyEdge(event.at("edge_id"), event.at("patch"));
        return json{{"done", true}};
    }
    // Unknown event type: return an error JSON
    return json{{"error", "unknown_event_type"}};
}

// Custom pretty-printer that keeps objects pretty-printed but forces
// the arrays stored under the key "path" to be output on a single line.
static std::string formatJson(const json& j, int indent = 0, const std::string& key = "") {
    std::string ind(indent, ' ');
    if (j.is_object()) {
        std::string out = "{\n";
        bool first = true;
        for (auto it = j.begin(); it != j.end(); ++it) {
            if (!first) out += ",\n";
            first = false;
            out += std::string(indent + 4, ' ') + "\"" + it.key() + "\": ";
            out += formatJson(it.value(), indent + 4, it.key());
        }
        out += "\n" + ind + "}";
        return out;
    }
    else if (j.is_array()) {
        
        if (key == "path" || key == "nodes") {
        return j.dump();
        }
        if (j.empty()) return "[]";
        std::string out = "[\n";
        for (size_t i = 0; i < j.size(); ++i) {
            if (i) out += ",\n";
            out += std::string(indent + 4, ' ') + formatJson(j[i], indent + 4, "");
        }
        out += "\n" + ind + "]";
        return out;
    }
    // primitives and strings: use dump() to get correct quoting
    return j.dump();
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json>" << std::endl;
        return 1;
    }
    std::string graphJsonPath = argv[1];
    std::string queriesJsonPath = argv[2];
    try {
        // --- 1. Load Graph ---
        std::ifstream graphFile(graphJsonPath);
        if (!graphFile.is_open()) {
            std::cerr << "Failed to open " << graphJsonPath << std::endl;
            return 1;
        }
        json graphData = json::parse(graphFile);
        graph.loadFromJson(graphData);
        std::ifstream queriesFile(queriesJsonPath);
        if (!queriesFile.is_open()) {
            std::cerr << "Failed to open " << queriesJsonPath << std::endl;
            return 1;
        }
        json queryData = json::parse(queriesFile);
        json outputEvents = json::array();

        for (const auto& event : queryData.at("events")) {
            auto start_time = std::chrono::high_resolution_clock::now();

            json result = processEvent(event);

            auto end_time = std::chrono::high_resolution_clock::now();
            result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

            outputEvents.push_back(result);
        }
        std::ofstream outputFile("output.json");
        if (!outputFile.is_open()) {
            std::cerr << "Failed to open output.json for writing" << std::endl;
            return 1;
        }
        // Write pretty JSON but keep `path` arrays inline
        outputFile << formatJson(outputEvents) << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "An error occurred: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
