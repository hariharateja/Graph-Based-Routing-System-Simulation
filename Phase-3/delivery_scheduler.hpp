#pragma once
#include "../Phase-1/graph.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <map>

using json = nlohmann::json;

struct Order {
    int id;
    int pickup_node;
    int dropoff_node;
};

struct ScheduleMetrics {
    double total_delivery_time_s = 0.0;
    double max_delivery_time_s = 0.0;
};

struct DriverState {
    int id;
    int current_node;
    double current_time;
    std::vector<int> carried_orders; // IDs of orders currently in the bag
    std::vector<int> route_path;     // History of nodes visited
    std::vector<int> delivered_orders_history; // delivered order ids history 
};

struct OrderStatus {
    int id;
    int pickup;
    int dropoff;
    bool is_picked_up = false;
    bool is_delivered = false;
    double completion_time = 0.0;
};

json solve_delivery_scheduling(const Graph& graph, const json& query);    