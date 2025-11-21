#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include "processquery.hpp"  // your dispatcher for phase-2 queries

using json = nlohmann::json;

// Graph is assumed to be defined in some other translation unit
// (e.g., graph.cpp), so we declare it here.
extern Graph graph;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <graph.json> <queries.json> <output.json>\n";
        return 1;
    }

    const std::string graphJsonPath   = argv[1];
    const std::string queriesJsonPath = argv[2];
    const std::string outputJsonPath  = argv[3];


    // 1. Read and load graph
    std::ifstream graphFile(graphJsonPath);
    if (!graphFile.is_open()) {
        std::cerr << "Failed to open " << graphJsonPath << "\n";
        return 1;
    }

    json graphData;
    try {
        graphData = json::parse(graphFile);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing graph JSON: " << e.what() << "\n";
        return 1;
    }
    graphFile.close();

    graph.loadFromJson(graphData);


    // 2. Read queries
    std::ifstream queriesFile(queriesJsonPath);
    if (!queriesFile.is_open()) {
        std::cerr << "Failed to open " << queriesJsonPath << "\n";
        return 1;
    }

    json queriesJson;
    try {
        queriesFile >> queriesJson;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing queries JSON: " << e.what() << "\n";
        return 1;
    }
    queriesFile.close();

    // Phase-2 spec doesn’t rigidly say "events" vs "queries"
    // at the top level, so we support both:
    const json* queryListPtr = nullptr;
    if (queriesJson.contains("events")) {
        queryListPtr = &queriesJson.at("events");
    } else if (queriesJson.contains("queries")) {
        queryListPtr = &queriesJson.at("queries");
    } else {
        std::cerr << "Error: queries JSON must contain 'events' or 'queries' array\n";
        return 1;
    }

    if (!queryListPtr->is_array()) {
        std::cerr << "Error: 'events' or 'queries' must be a JSON array\n";
        return 1;
    }

    const json& queryList = *queryListPtr;
    std::vector<json> results;
    results.reserve(queryList.size());


    // 3. Process queries one by one
    for (const auto& query : queryList) {
        auto start_time = std::chrono::high_resolution_clock::now();

        json result;
        try {
            result = process_query(query);  // dispatch based on query["type"]
        } catch (const std::exception& e) {
            // In case of error, log and create a minimal failure object
            std::cerr << "Error processing query id="
                      << (query.contains("id") ? query["id"] : json(-1))
                      << ": " << e.what() << "\n";

            result["id"] = query.value("id", -1);
            result["error"] = std::string("processing_failed");
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Spec: driver adds "processing_time" (in ms) to each output
        result["processing_time"] = ms;
        results.push_back(std::move(result));
    }

    // 4. Build output JSON
    json output;

    // Copy meta.id from input if present, else default to "qset2" or similar
    if (queriesJson.contains("meta") && queriesJson["meta"].contains("id")) {
        output["meta"]["id"] = queriesJson["meta"]["id"];
    } else {
        output["meta"]["id"] = "qset2";
    }

    output["results"] = results;

    // 5. Write output.json
    std::ofstream outFile(outputJsonPath);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open " << outputJsonPath << " for writing\n";
        return 1;
    }

    outFile << output.dump(4) << "\n";
    outFile.close();

    return 0;
}