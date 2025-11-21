#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "delivery_scheduler.hpp"
#include "../Phase-1/graph.hpp"

using json = nlohmann::json;

Graph graph; // global graph object
int main(int argc, char* argv[]){
    if (argc != 4){
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }

    std::string graphJsonPath = argv[1];
    std::string queriesJsonPath = argv[2];
    std::string outputJsonPath = argv[3];

    std::ifstream graphFile(graphJsonPath);

    if (!graphFile.is_open()){
        std::cerr << "Failed to open " << graphJsonPath << std::endl;
        return 1;
    }
    json graphData = json::parse(graphFile);
    graph.loadFromJson(graphData);
    graphFile.close();

    std::ifstream queries_file(queriesJsonPath);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << queriesJsonPath << std::endl;
        return 1;
    }

    json input_json;
    queries_file >> input_json;
    queries_file.close();

    auto start_time = std::chrono::high_resolution_clock::now();

    json output = solve_delivery_scheduling(graph, input_json);
    
    auto end_time = std::chrono::high_resolution_clock::now();

    std::ofstream output_file(outputJsonPath);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    output_file << output.dump(4) << std::endl;
    output_file.close();

    return 0;
}