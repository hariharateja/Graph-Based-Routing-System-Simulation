#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "processquery.hpp"

using json = nlohmann::json;
extern Graph graph;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <graph.json> <queries.json> <output.json>" << std::endl;
        return 1;
    }
    
    std::string graphJsonPath = argv[1];
    std::string queriesJsonPath = argv[2];
    
    // Read and load graph
    std::ifstream graphFile(graphJsonPath);
    if (!graphFile.is_open()) {
        std::cerr << "Failed to open " << graphJsonPath << std::endl;
        return 1;
    }
    json graphData = json::parse(graphFile);
    graph.loadFromJson(graphData);
    graphFile.close();
    
    // Read queries
    std::ifstream queries_file(queriesJsonPath);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << queriesJsonPath << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;
    queries_file.close();

    std::vector<json> results;

    for (const auto& query : queries_json.at("events")) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        results.push_back(result);
    }
    
    // Create output in the required format
    json output;
    
    // Extract the query set id from the queries_json meta field
    if (queries_json.contains("meta") && queries_json["meta"].contains("id")) {
        output["meta"]["id"] = queries_json["meta"]["id"];
    } else {
        output["meta"]["id"] = "qset1";  // default fallback
    }
    
    output["results"] = results;
    
    // Write to output file
    std::ofstream output_file(argv[3]);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open output.json for writing" << std::endl;
        return 1;
    }

    output_file << output.dump(4) << std::endl;
    output_file.close();
    
    return 0;
}