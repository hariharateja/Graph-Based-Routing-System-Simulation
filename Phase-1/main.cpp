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
    // Read graph from first file
    /*
        Add your graph reading and processing code here
        Initialize any classes and data structures needed for query processing
    */
    std::string graphJsonPath = argv[1];
    std::string queriesJsonPath = argv[2];
    std::ifstream graphFile(graphJsonPath);
    if (!graphFile.is_open()) {
        std::cerr << "Failed to open " << graphJsonPath << std::endl;
        return 1;
    }
    json graphData = json::parse(graphFile);
    graph.loadFromJson(graphData);
    std::ifstream queries_file(queriesJsonPath);
    if (!queries_file.is_open()) {
        std::cerr << "Failed to open " << queriesJsonPath << std::endl;
        return 1;
    }
    json queries_json;
    queries_file >> queries_json;

    std::vector<json> results;

    for (const auto& query : queries_json.at("events")) {
        auto start_time = std::chrono::high_resolution_clock::now();
        /*
            Add your query processing code here
            Each query should return a json object which should be printed to sample.json
        */

        // Answer each query replacing the function process_query using 
        // whatever function or class methods that you have implemented

        json result = process_query(query);

        auto end_time = std::chrono::high_resolution_clock::now();
        result["processing_time"] = std::chrono::duration<double, std::milli>(end_time - start_time).count();

            results.push_back(result);
        }
        std::ofstream output_file(argv[3]);
        if (!output_file.is_open()) {
            std::cerr << "Failed to open output.json for writing" << std::endl;
            return 1;
        }

        json output = results;
        output_file << output.dump(4) << std::endl;
        
        output_file.close();
        return 0;

}
